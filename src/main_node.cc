#include "ros/ros.h"

#include "code_it_pr2/robot_api.h"
#include "rapid/pr2/pr2.h"

using code_it_pr2::RobotApi;

int main(int argc, char** argv) {
  ros::init(argc, argv, "code_it_pr2");
  ros::NodeHandle nh;

  rapid::pr2::Pr2 robot = rapid::pr2::BuildReal();
  RobotApi api(robot);
  ros::ServiceServer say_srv =
      nh.advertiseService("code_it/api/say", &RobotApi::Say, &api);
  ros::ServiceServer disp_msg_srv = nh.advertiseService(
      "code_it/api/display_message", &RobotApi::DisplayMessage, &api);
  ros::ServiceServer ask_mc_srv = nh.advertiseService(
      "code_it/api/ask_multiple_choice", &RobotApi::AskMultipleChoice, &api);
  ros::ServiceServer look_at_srv =
      nh.advertiseService("code_it/api/look_at", &RobotApi::LookAt, &api);
  ros::Subscriber stop_sub = nh.subscribe(
      "code_it/is_program_running", 10, &RobotApi::HandleProgramStopped, &api);
  // TODO(jstn): A multi-threaded spinner may be needed for action clients to
  // work.
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  return 0;
}
