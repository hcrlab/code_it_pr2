#include "ros/ros.h"

#include "boost/shared_ptr.hpp"
#include "code_it_pr2/robot_api.h"
#include "rapid_pr2/pr2.h"
#include "std_msgs/String.h"

using boost::shared_ptr;
using code_it_pr2::RobotApi;
using rapid::pr2::Pr2;

int main(int argc, char** argv) {
  ros::init(argc, argv, "code_it_pr2");
  ros::NodeHandle nh;
  ros::Publisher error_pub =
      nh.advertise<std_msgs::String>("code_it/errors", 10);

  shared_ptr<Pr2> robot = rapid::pr2::BuildReal();
  RobotApi api(robot, error_pub);

  ros::ServiceServer ask_mc_srv = nh.advertiseService(
      "code_it/api/ask_multiple_choice", &RobotApi::AskMultipleChoice, &api);
  ros::ServiceServer disp_msg_srv = nh.advertiseService(
      "code_it/api/display_message", &RobotApi::DisplayMessage, &api);
  ros::ServiceServer find_objects_srv = nh.advertiseService(
      "code_it/api/find_objects", &RobotApi::FindObjects, &api);
  ros::ServiceServer look_at_srv =
      nh.advertiseService("code_it/api/look_at", &RobotApi::LookAt, &api);
  ros::ServiceServer pick_srv =
      nh.advertiseService("code_it/api/pick", &RobotApi::Pick, &api);
  ros::ServiceServer place_srv =
      nh.advertiseService("code_it/api/place", &RobotApi::Place, &api);
  ros::ServiceServer say_srv =
      nh.advertiseService("code_it/api/say", &RobotApi::Say, &api);
  ros::ServiceServer tuck_arms_srv =
      nh.advertiseService("code_it/api/tuck_arms", &RobotApi::TuckArms, &api);
  ros::Subscriber stop_sub = nh.subscribe(
      "code_it/is_program_running", 10, &RobotApi::HandleProgramStopped, &api);
  ros::spin();
  return 0;
}
