#include "code_it_pr2/robot_api.h"

#include "code_it/AskMultipleChoice.h"
#include "code_it/DisplayMessage.h"
#include "code_it/LookAt.h"
#include "code_it/Say.h"
#include "rapid/pr2/pr2.h"
#include "std_msgs/Bool.h"

namespace code_it_pr2 {
RobotApi::RobotApi(const rapid::pr2::Pr2& robot) : robot_(robot) {}
bool RobotApi::Say(code_it::SayRequest& req, code_it::SayResponse& res) {
  robot_.sound.Say(req.text);
  return true;
}

bool RobotApi::AskMultipleChoice(code_it::AskMultipleChoiceRequest& req,
                                 code_it::AskMultipleChoiceResponse& res) {
  std::string choice;
  bool success =
      robot_.display.AskMultipleChoice(req.question, req.choices, &choice);
  res.choice = choice;
  return success;
}

bool RobotApi::DisplayMessage(code_it::DisplayMessageRequest& req,
                              code_it::DisplayMessageResponse& res) {
  return robot_.display.ShowMessage(req.h1_text, req.h2_text);
}

bool RobotApi::LookAt(code_it::LookAtRequest& req,
                      code_it::LookAtResponse& res) {
  return robot_.head.LookAt(req.target);
}

void RobotApi::HandleProgramStopped(const std_msgs::Bool& msg) {
  if (msg.data) {
    return;  // Program is running, nothing to do.
  }
  bool success = robot_.display.ShowDefault();
  if (!success) {
    ROS_ERROR("Failed to reset screen on program stop.");
  }
}
}  // namespace code_it_pr2
