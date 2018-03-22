#include "ros/ros.h"

#include "actionlib/client/simple_action_client.h"
#include "blinky/FaceAction.h"
#include "boost/shared_ptr.hpp"
#include "code_it_pr2/robot_api.h"
#include "object_search_msgs/GetObjectInfo.h"
#include "object_search_msgs/SearchFromDb.h"
#include "pr2_pbd_interaction/ExecuteAction.h"
#include "rapid_pr2/pr2.h"
#include "rapid_ros/action_client.h"
#include "rapid_ros/publisher.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"

using boost::shared_ptr;
using code_it_pr2::RobotApi;
using pr2_pbd_interaction::ExecuteAction;
using rapid::pr2::Pr2;
using visualization_msgs::Marker;

int main(int argc, char** argv) {
  ros::init(argc, argv, "code_it_pr2");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  rapid_ros::Publisher<Marker> marker_pub(
      nh.advertise<Marker>("code_it_markers", 100));
  rapid_ros::ActionClient<ExecuteAction> pbd_client("execute_pbd_action");
  ros::ServiceClient find_landmark =
      nh.serviceClient<object_search_msgs::SearchFromDb>("find_object_from_db");
  ros::ServiceClient get_landmark_info =
      nh.serviceClient<object_search_msgs::GetObjectInfo>("get_object_info");
  actionlib::SimpleActionClient<blinky::FaceAction> blinky_client("blinky",
                                                                  true);
  if (!blinky_client.waitForServer(ros::Duration(5.0))) {
    ROS_ERROR("Blinky server not available!");
  }

  Pr2* robot = rapid::pr2::BuildReal(nh);
  RobotApi api(robot, marker_pub, pbd_client, find_landmark, get_landmark_info,
               &blinky_client);

  ros::ServiceServer ask_mc_srv = nh.advertiseService(
      "code_it/api/ask_multiple_choice", &RobotApi::AskMultipleChoice, &api);
  ros::ServiceServer disp_msg_srv = nh.advertiseService(
      "code_it/api/display_message", &RobotApi::DisplayMessage, &api);
  ros::ServiceServer find_custom_landmarks_srv = nh.advertiseService(
      "code_it/api/find_custom_landmark", &RobotApi::FindCustomLandmarks, &api);
  ros::ServiceServer find_objects_srv = nh.advertiseService(
      "code_it/api/find_objects", &RobotApi::FindObjects, &api);
  ros::ServiceServer is_gripper_open_srv = nh.advertiseService(
      "code_it/api/is_gripper_open", &RobotApi::IsGripperOpen, &api);
  ros::ServiceServer look_at_srv =
      nh.advertiseService("code_it/api/look_at", &RobotApi::LookAt, &api);
  // ros::ServiceServer pick_srv =
  //    nh.advertiseService("code_it/api/pick", &RobotApi::Pick, &api);
  // ros::ServiceServer place_srv =
  //    nh.advertiseService("code_it/api/place", &RobotApi::Place, &api);
  ros::ServiceServer run_pbd_srv = nh.advertiseService(
      "code_it/api/run_pbd_action", &RobotApi::RunPbdAction, &api);
  ros::ServiceServer say_srv =
      nh.advertiseService("code_it/api/say", &RobotApi::Say, &api);
  ros::ServiceServer set_gripper_srv = nh.advertiseService(
      "code_it/api/set_gripper", &RobotApi::SetGripper, &api);
  ros::ServiceServer tuck_arms_srv =
      nh.advertiseService("code_it/api/tuck_arms", &RobotApi::TuckArms, &api);
  ros::Subscriber stop_sub = nh.subscribe(
      "code_it/is_program_running", 10, &RobotApi::HandleProgramStopped, &api);
  ROS_INFO("CodeIt! for the PR2 is ready.");
  ros::waitForShutdown();
  delete robot;
  return 0;
}
