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
  ros::Subscriber stop_sub = nh.subscribe(
      "code_it/stopped", 10, &RobotApi::HandleProgramStopped, &api);
  ros::spin();
  return 0;
}
