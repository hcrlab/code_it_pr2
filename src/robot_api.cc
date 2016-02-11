#include "code_it_pr2/robot_api.h"

#include "code_it/Say.h"
#include "rapid/pr2/pr2.h"
#include "std_msgs/Empty.h"

namespace code_it_pr2 {
RobotApi::RobotApi(const rapid::pr2::Pr2& robot) : robot_(robot) {}
bool RobotApi::Say(code_it::SayRequest& req, code_it::SayResponse& res) {
  robot_.Say(req.text);
  return true;
}

void RobotApi::HandleProgramStopped(const std_msgs::Empty& msg) { return; }
}  // namespace code_it_pr2
