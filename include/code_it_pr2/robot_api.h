#ifndef _CODE_IT_PR2_ROBOT_API_H_
#define _CODE_IT_PR2_ROBOT_API_H_

#include "code_it_msgs/AskMultipleChoice.h"
#include "code_it_msgs/DisplayMessage.h"
#include "code_it_msgs/LookAt.h"
#include "code_it_msgs/Say.h"
#include "rapid/pr2/pr2.h"
#include "std_msgs/Bool.h"

namespace code_it_pr2 {
class RobotApi {
 public:
  RobotApi(const rapid::pr2::Pr2& robot);
  bool AskMultipleChoice(code_it_msgs::AskMultipleChoiceRequest& req,
                         code_it_msgs::AskMultipleChoiceResponse& res);
  bool DisplayMessage(code_it_msgs::DisplayMessageRequest& req,
                      code_it_msgs::DisplayMessageResponse& res);
  bool LookAt(code_it_msgs::LookAtRequest& req,
              code_it_msgs::LookAtResponse& res);
  bool Say(code_it_msgs::SayRequest& req, code_it_msgs::SayResponse& res);
  void HandleProgramStopped(const std_msgs::Bool& msg);

 private:
  rapid::pr2::Pr2 robot_;
};
}  // namespace code_it_pr2
#endif  // _CODE_IT_PR2_ROBOT_API_H_
