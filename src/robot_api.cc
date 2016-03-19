#include "code_it_pr2/robot_api.h"

#include <vector>

#include "boost/shared_ptr.hpp"
#include "code_it_msgs/AskMultipleChoice.h"
#include "code_it_msgs/DisplayMessage.h"
#include "code_it_msgs/LookAt.h"
#include "code_it_msgs/Say.h"
#include "rapid_perception/pr2.h"
#include "rapid_perception/rgbd.hpp"
#include "rapid_pr2/pr2.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"

namespace rpe = rapid::perception;
using boost::shared_ptr;

namespace code_it_pr2 {
RobotApi::RobotApi(shared_ptr<rapid::pr2::Pr2> robot) : robot_(robot) {}
bool RobotApi::Say(code_it_msgs::SayRequest& req,
                   code_it_msgs::SayResponse& res) {
  robot_->sound.Say(req.text);
  return true;
}

bool RobotApi::AskMultipleChoice(code_it_msgs::AskMultipleChoiceRequest& req,
                                 code_it_msgs::AskMultipleChoiceResponse& res) {
  std::string choice;
  bool success =
      robot_->display.AskMultipleChoice(req.question, req.choices, &choice);
  res.choice = choice;
  return success;
}

bool RobotApi::DisplayMessage(code_it_msgs::DisplayMessageRequest& req,
                              code_it_msgs::DisplayMessageResponse& res) {
  return robot_->display.ShowMessage(req.h1_text, req.h2_text);
}

bool RobotApi::FindObjects(code_it_msgs::FindObjectsRequest& req,
                           code_it_msgs::FindObjectsResponse& res) {
  rpe::Scene scene;
  bool success = rpe::pr2::GetManipulationScene(&scene);
  if (!success) {
    return false;
  }
  boost::shared_ptr<rpe::Tabletop> tt = scene.GetPrimarySurface();
  std::vector<rpe::Object> objects = tt->objects();
  for (size_t i = 0; i < objects.size(); ++i) {
    const rpe::Object& obj = objects[i];
    code_it_msgs::Object msg;
    msg.pose = obj.pose();
    msg.scale = obj.scale();
    res.objects.push_back(msg);
  }
  return true;
}

bool RobotApi::LookAt(code_it_msgs::LookAtRequest& req,
                      code_it_msgs::LookAtResponse& res) {
  return robot_->head.LookAt(req.target);
}

void RobotApi::HandleProgramStopped(const std_msgs::Bool& msg) {
  if (msg.data) {
    return;  // Program is running, nothing to do.
  }
  bool success = robot_->display.ShowDefault();
  if (!success) {
    ROS_ERROR("Failed to reset screen on program stop.");
  }
}
}  // namespace code_it_pr2
