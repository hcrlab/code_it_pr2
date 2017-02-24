#ifndef _CODE_IT_PR2_ROBOT_API_H_
#define _CODE_IT_PR2_ROBOT_API_H_

#include <string>

#include "code_it_msgs/AskMultipleChoice.h"
#include "code_it_msgs/DisplayMessage.h"
#include "code_it_msgs/FindCustomLandmarks.h"
#include "code_it_msgs/FindObjects.h"
#include "code_it_msgs/IsGripperOpen.h"
#include "code_it_msgs/LookAt.h"
//#include "code_it_msgs/Pick.h"
//#include "code_it_msgs/Place.h"
#include "code_it_msgs/RunPbdAction.h"
#include "code_it_msgs/Say.h"
#include "code_it_msgs/SetGripper.h"
#include "code_it_msgs/TuckArms.h"
#include "object_search_msgs/SearchFromDb.h"
#include "pr2_pbd_interaction/ExecuteAction.h"
#include "rapid_perception/scene.h"
#include "rapid_perception/scene_viz.h"
#include "rapid_pr2/pr2.h"
#include "rapid_ros/publisher.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"

namespace code_it_pr2 {
namespace errors {
static const char ASK_MC_QUESTION[] = "Failed to ask multiple choice question.";
static const char DISPLAY_MESSAGE[] = "Failed to display message.";
static const char GET_SCENE[] = "The robot failed to read its camera data.";
static const char FIND_LANDMARK_FAILED[] =
    "The robot was unable to find the landmark.";
static const char LOOK_AT[] = "Failed to look at target.";
static const char IS_OPEN_AMBIG[] = "Not sure which gripper to check is open.";
static const char PBD_ACTION_FAILED[] = "The PbD action failed to run.";
static const char PBD_ACTION_TIMED_OUT[] = "The PbD action timed out.";
static const char PICK_OBJECT[] = "The robot was unable to pick up the object.";
static const char PICK_OBJECT_NOT_FOUND[] = "The object to pick was not found.";
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
static const char TUCK_DEPLOY[] =
    "The robot was unable to tuck or deploy its arms.";
static const char PICK_SCENE_NOT_PARSED[] =
    "The robot needs to find objects before it can pick anything.";
}  // namespace errors

class RobotApi {
 public:
  // Does not take ownership of the Pr2 pointer.
  RobotApi(
      rapid::pr2::Pr2* robot,
      const rapid_ros::Publisher<visualization_msgs::Marker>& marker_pub,
      rapid_ros::ActionClient<pr2_pbd_interaction::ExecuteAction>& pbd_client,
      const ros::ServiceClient& find_landmark,
      const ros::ServiceClient& get_landmark_info);
  bool AskMultipleChoice(code_it_msgs::AskMultipleChoiceRequest& req,
                         code_it_msgs::AskMultipleChoiceResponse& res);
  bool DisplayMessage(code_it_msgs::DisplayMessageRequest& req,
                      code_it_msgs::DisplayMessageResponse& res);
  bool FindCustomLandmarks(code_it_msgs::FindCustomLandmarksRequest& req,
                           code_it_msgs::FindCustomLandmarksResponse& res);
  bool FindObjects(code_it_msgs::FindObjectsRequest& req,
                   code_it_msgs::FindObjectsResponse& res);
  bool IsGripperOpen(code_it_msgs::IsGripperOpenRequest& req,
                     code_it_msgs::IsGripperOpenResponse& res);
  bool LookAt(code_it_msgs::LookAtRequest& req,
              code_it_msgs::LookAtResponse& res);
  // bool Pick(code_it_msgs::PickRequest& req, code_it_msgs::PickResponse& res);
  // bool Place(code_it_msgs::PlaceRequest& req, code_it_msgs::PlaceResponse&
  // res);
  bool RunPbdAction(code_it_msgs::RunPbdActionRequest& req,
                    code_it_msgs::RunPbdActionResponse& res);
  bool Say(code_it_msgs::SayRequest& req, code_it_msgs::SayResponse& res);
  bool SetGripper(code_it_msgs::SetGripperRequest& req,
                  code_it_msgs::SetGripperResponse& res);
  bool TuckArms(code_it_msgs::TuckArmsRequest& req,
                code_it_msgs::TuckArmsResponse& res);
  void HandleProgramStopped(const std_msgs::Bool& msg);

 private:
  rapid::pr2::Pr2* const robot_;
  tf::TransformListener tf_listener_;
  rapid_ros::Publisher<visualization_msgs::Marker> marker_pub_;
  rapid::perception::Scene scene_;         // Most recent scene parsed.
  rapid::perception::SceneViz scene_viz_;  // Most recent scene parsed.
  bool scene_has_parsed_;
  rapid_ros::ActionClient<pr2_pbd_interaction::ExecuteAction>& pbd_client_;
  ros::ServiceClient find_landmark_;
  ros::ServiceClient get_landmark_info_;
};
}  // namespace code_it_pr2
#endif  // _CODE_IT_PR2_ROBOT_API_H_
