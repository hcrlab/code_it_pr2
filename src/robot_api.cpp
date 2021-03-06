#include "code_it_pr2/robot_api.h"

#include <string>
#include <vector>

#include "boost/shared_ptr.hpp"
#include "code_it_msgs/ArmId.h"
#include "code_it_msgs/AskMultipleChoice.h"
#include "code_it_msgs/DisplayMessage.h"
#include "code_it_msgs/FindCustomLandmarks.h"
#include "code_it_msgs/FindObjects.h"
#include "code_it_msgs/GripperId.h"
#include "code_it_msgs/Landmark.h"
#include "code_it_msgs/LookAt.h"
#include "code_it_msgs/Say.h"
#include "code_it_msgs/SetGripper.h"
#include "code_it_msgs/TuckArms.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "object_search_msgs/GetObjectInfo.h"
#include "object_search_msgs/Match.h"
#include "object_search_msgs/SearchFromDb.h"
#include "pr2_pbd_interaction/ExecuteAction.h"
#include "pr2_pbd_interaction/Landmark.h"
#include "rapid_perception/pr2.h"
#include "rapid_perception/scene.h"
#include "rapid_perception/scene_viz.h"
#include "rapid_pr2/pr2.h"
#include "rapid_ros/publisher.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"

namespace rpe = rapid::perception;
using boost::shared_ptr;
using pr2_pbd_interaction::ExecuteAction;
using pr2_pbd_interaction::ExecuteGoal;
using pr2_pbd_interaction::ExecuteResult;
using std::string;
using visualization_msgs::Marker;
typedef actionlib::SimpleActionClient<blinky::FaceAction> BlinkyClient;

namespace code_it_pr2 {
RobotApi::RobotApi(rapid::pr2::Pr2* robot,
                   const rapid_ros::Publisher<Marker>& marker_pub,
                   rapid_ros::ActionClient<ExecuteAction>& pbd_client,
                   const ros::ServiceClient& find_landmark,
                   const ros::ServiceClient& get_landmark_info,
                   BlinkyClient* blinky_client)
    : robot_(robot),
      tf_listener_(),
      marker_pub_(marker_pub),
      scene_(),
      scene_viz_(&marker_pub_),
      scene_has_parsed_(false),
      pbd_client_(pbd_client),
      find_landmark_(find_landmark),
      get_landmark_info_(get_landmark_info),
      blinky_client_(blinky_client) {}

bool RobotApi::AskMultipleChoice(code_it_msgs::AskMultipleChoiceRequest& req,
                                 code_it_msgs::AskMultipleChoiceResponse& res) {
  blinky::FaceGoal goal;
  goal.display_type = blinky::FaceGoal::ASK_CHOICE;
  goal.question = req.question;
  goal.choices = req.choices;
  if (!blinky_client_->waitForServer(ros::Duration(5.0))) {
    res.error = errors::BLINKY_NOT_AVAILABLE;
    return true;
  }
  blinky_client_->sendGoal(goal);
  blinky_client_->waitForResult(ros::Duration(0));
  blinky::FaceResultConstPtr result = blinky_client_->getResult();
  res.choice = result->choice;
  return true;
}

bool RobotApi::DisplayMessage(code_it_msgs::DisplayMessageRequest& req,
                              code_it_msgs::DisplayMessageResponse& res) {
  blinky::FaceGoal goal;
  goal.display_type = blinky::FaceGoal::DISPLAY_MESSAGE;
  goal.h1_text = req.h1_text;
  goal.h2_text = req.h2_text;
  blinky_client_->sendGoal(goal);
  if (!blinky_client_->waitForResult(ros::Duration(5.0))) {
    res.error = errors::BLINKY_NOT_AVAILABLE;
    return true;
  }

  return true;
}

bool RobotApi::FindCustomLandmarks(
    code_it_msgs::FindCustomLandmarksRequest& req,
    code_it_msgs::FindCustomLandmarksResponse& res) {
  object_search_msgs::GetObjectInfoRequest info_req;
  info_req.db_id = req.db_id;
  info_req.name = req.name;
  object_search_msgs::GetObjectInfoResponse info_res;
  get_landmark_info_.call(info_req, info_res);

  object_search_msgs::SearchFromDbRequest search_req;
  search_req.object_id = req.db_id;
  search_req.name = req.name;
  search_req.min_results = 0;
  search_req.is_tabletop = req.is_tabletop;
  search_req.max_error = 0;  // 0 value means get from parameter server.
  object_search_msgs::SearchFromDbResponse search_res;
  bool success = find_landmark_.call(search_req, search_res);
  if (!success) {
    res.error = errors::FIND_LANDMARK_FAILED;
    return true;
  }
  // Convert object_search Matches to code_it Landmarks.
  for (size_t i = 0; i < search_res.matches.size(); ++i) {
    const object_search_msgs::Match& match = search_res.matches[i];
    code_it_msgs::Landmark landmark;
    landmark.name = info_res.name;
    landmark.pose.pose = match.pose;
    landmark.scale = info_res.dimensions;
    landmark.db_id = req.db_id;
    res.landmarks.push_back(landmark);
  }

  return true;
}

bool RobotApi::FindObjects(code_it_msgs::FindObjectsRequest& req,
                           code_it_msgs::FindObjectsResponse& res) {
  bool success = rpe::pr2::GetManipulationScene(tf_listener_, &scene_);
  if (!success) {
    res.error = errors::GET_SCENE;
    return true;
  }
  scene_viz_.set_scene(scene_);
  scene_viz_.Visualize();
  scene_has_parsed_ = true;
  rpe::HSurface tt = scene_.primary_surface();
  const std::vector<rpe::Object>& objects = tt.objects();
  for (size_t i = 0; i < objects.size(); ++i) {
    const rpe::Object& obj = objects[i];
    code_it_msgs::Object msg;
    msg.name = obj.name();
    msg.pose = obj.pose();
    msg.scale = obj.scale();
    res.objects.push_back(msg);
  }
  return true;
}

bool RobotApi::IsGripperOpen(code_it_msgs::IsGripperOpenRequest& req,
                             code_it_msgs::IsGripperOpenResponse& res) {
  if (req.gripper.id == code_it_msgs::GripperId::RIGHT) {
    res.is_open = robot_->right_gripper()->IsOpen();
  } else if (req.gripper.id == code_it_msgs::GripperId::LEFT) {
    res.is_open = robot_->left_gripper()->IsOpen();
  } else {
    res.error = errors::IS_OPEN_AMBIG;
    return true;
  }
  return true;
}

bool RobotApi::LookAt(code_it_msgs::LookAtRequest& req,
                      code_it_msgs::LookAtResponse& res) {
  bool success = robot_->head()->LookAt(req.target);
  if (!success) {
    res.error = errors::LOOK_AT;
  }
  return true;
}

bool RobotApi::RunPbdAction(code_it_msgs::RunPbdActionRequest& req,
                            code_it_msgs::RunPbdActionResponse& res) {
  ExecuteGoal goal;
  goal.action_id = req.action_id;
  goal.name = req.name;

  // Convert code_it landmarks to PbD landmarks.
  for (size_t i = 0; i < req.landmarks.size(); ++i) {
    pr2_pbd_interaction::Landmark landmark;
    landmark.name = req.landmarks[i].name;
    landmark.pose = req.landmarks[i].pose.pose;
    landmark.dimensions = req.landmarks[i].scale;
    landmark.db_id = req.landmarks[i].db_id;
    goal.landmarks.push_back(landmark);
  }

  pbd_client_.sendGoal(goal);
  bool on_time = pbd_client_.waitForResult();
  if (!on_time) {
    res.error = errors::PBD_ACTION_TIMED_OUT;
    return true;
  }
  actionlib::SimpleClientGoalState state = pbd_client_.getState();
  if (state == state.SUCCEEDED) {
  } else if (state == state.ABORTED) {
    res.error = "Execution aborted: " + state.getText();
  } else {
    res.error = errors::PBD_ACTION_FAILED;
  }
  ExecuteResult::ConstPtr result = pbd_client_.getResult();

  return true;
}

bool RobotApi::Say(code_it_msgs::SayRequest& req,
                   code_it_msgs::SayResponse& res) {
  robot_->sound()->Say(req.text);
  return true;
}

bool RobotApi::SetGripper(code_it_msgs::SetGripperRequest& req,
                          code_it_msgs::SetGripperResponse& res) {
  if (req.gripper.id == code_it_msgs::GripperId::LEFT) {
    if (req.action == code_it_msgs::SetGripperRequest::OPEN) {
      robot_->left_gripper()->Open(req.max_effort);
    } else if (req.action == code_it_msgs::SetGripperRequest::CLOSE) {
      robot_->left_gripper()->Close(req.max_effort);
    }
  } else {
    if (req.action == code_it_msgs::SetGripperRequest::OPEN) {
      robot_->right_gripper()->Open(req.max_effort);
    } else if (req.action == code_it_msgs::SetGripperRequest::CLOSE) {
      robot_->right_gripper()->Close(req.max_effort);
    }
  }
  return true;
}

bool RobotApi::TuckArms(code_it_msgs::TuckArmsRequest& req,
                        code_it_msgs::TuckArmsResponse& res) {
  bool success = true;
  if (req.tuck_left && req.tuck_right) {
    success = robot_->tuck_arms()->TuckArms();
  } else if (req.tuck_left && !req.tuck_right) {
    success = robot_->tuck_arms()->DeployRight();
  } else if (!req.tuck_left && req.tuck_right) {
    success = robot_->tuck_arms()->DeployLeft();
  } else {
    success = robot_->tuck_arms()->DeployArms();
  }
  if (!success) {
    res.error = errors::TUCK_DEPLOY;
    return true;
  }
  return true;
}

void RobotApi::HandleProgramStopped(const std_msgs::Bool& msg) {
  if (msg.data) {
    return;  // Program is running, nothing to do.
  }
  blinky::FaceGoal goal;
  goal.display_type = blinky::FaceGoal::DEFAULT;
  blinky_client_->sendGoal(goal);
}
}  // namespace code_it_pr2
