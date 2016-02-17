#ifndef _CODE_IT_PR2_ROBOT_API_H_
#define _CODE_IT_PR2_ROBOT_API_H_

#include "code_it/AskMultipleChoice.h"
#include "code_it/DisplayMessage.h"
#include "code_it/LookAt.h"
#include "code_it/Say.h"
#include "rapid/pr2/pr2.h"
#include "std_msgs/Bool.h"

namespace code_it_pr2 {
class RobotApi {
 public:
  RobotApi(const rapid::pr2::Pr2& robot);
  bool AskMultipleChoice(code_it::AskMultipleChoiceRequest& req,
                         code_it::AskMultipleChoiceResponse& res);
  bool DisplayMessage(code_it::DisplayMessageRequest& req,
                      code_it::DisplayMessageResponse& res);
  bool LookAt(code_it::LookAtRequest& req, code_it::LookAtResponse& res);
  bool Say(code_it::SayRequest& req, code_it::SayResponse& res);
  void HandleProgramStopped(const std_msgs::Bool& msg);

 private:
  rapid::pr2::Pr2 robot_;
};
}  // namespace code_it_pr2
#endif  // _CODE_IT_PR2_ROBOT_API_H_
