#ifndef _CODE_IT_PR2_ROBOT_API_H_
#define _CODE_IT_PR2_ROBOT_API_H_

#include <string>

#include "boost/shared_ptr.hpp"
#include "code_it_msgs/AskMultipleChoice.h"
#include "code_it_msgs/DisplayMessage.h"
#include "code_it_msgs/FindObjects.h"
#include "code_it_msgs/LookAt.h"
#include "code_it_msgs/Pick.h"
#include "code_it_msgs/Place.h"
#include "code_it_msgs/Say.h"
#include "code_it_msgs/SetGripper.h"
#include "code_it_msgs/TuckArms.h"
#include "rapid_pr2/pr2.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "tf/transform_listener.h"

namespace code_it_pr2 {
namespace errors {
static const char ASK_MC_QUESTION[] =
    "Failed to update the screen to ask a question.";
static const char GET_SCENE[] = "The robot failed to read its camera data.";
static const char PICK_OBJECT[] = "The robot was unable to pick up the object.";
static const char PICK_LEFT_FULL[] =
    "The robot is already holding something in its left hand.";
static const char PICK_RIGHT_FULL[] =
    "The robot is already holding something in its right hand.";
static const char PICK_ARMS_FULL[] =
    "The robot is already holding something in both hands.";
static const char PLACE_OBJECT[] =
    "The robot was unable to place the object down.";
static const char PLACE_NO_OBJECTS[] = "The robot is not holding any objects.";
static const char PLACE_NO_LEFT_OBJECT[] =
    "The robot has no object in its left hand to place.";
static const char PLACE_NO_RIGHT_OBJECT[] =
    "The robot has no object in its right hand to place.";
static const char RESET_SCREEN_ON_STOP[] =
    "Failed to reset the screen on program stop.";
static const char TUCK_DEPLOY[] =
    "The robot was unable to tuck or deploy its arms.";
}  // namespace errors

class RobotApi {
 public:
  RobotApi(boost::shared_ptr<rapid::pr2::Pr2> robot,
           const ros::Publisher& error_pub);
  bool AskMultipleChoice(code_it_msgs::AskMultipleChoiceRequest& req,
                         code_it_msgs::AskMultipleChoiceResponse& res);
  bool DisplayMessage(code_it_msgs::DisplayMessageRequest& req,
                      code_it_msgs::DisplayMessageResponse& res);
  bool FindObjects(code_it_msgs::FindObjectsRequest& req,
                   code_it_msgs::FindObjectsResponse& res);
  bool LookAt(code_it_msgs::LookAtRequest& req,
              code_it_msgs::LookAtResponse& res);
  bool Pick(code_it_msgs::PickRequest& req, code_it_msgs::PickResponse& res);
  bool Place(code_it_msgs::PlaceRequest& req, code_it_msgs::PlaceResponse& res);
  bool Say(code_it_msgs::SayRequest& req, code_it_msgs::SayResponse& res);
  bool SetGripper(code_it_msgs::SetGripperRequest& req,
                  code_it_msgs::SetGripperResponse& res);
  bool TuckArms(code_it_msgs::TuckArmsRequest& req,
                code_it_msgs::TuckArmsResponse& res);
  void HandleProgramStopped(const std_msgs::Bool& msg);

 private:
  void PublishError(const std::string& error);
  boost::shared_ptr<rapid::pr2::Pr2> robot_;
  ros::Publisher error_pub_;
  tf::TransformListener tf_listener_;
};
}  // namespace code_it_pr2
#endif  // _CODE_IT_PR2_ROBOT_API_H_
