#include "code_it_pr2/robot_api.h"

#include "code_it/Say.h"
#include "code_it/AskMultipleChoice.h"
#include "code_it/DisplayMessage.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "rapid/display/display.h"
#include "rapid/pr2/pr2.h"
#include "rapid/sound/sound.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

using code_it_pr2::RobotApi;
using rapid::display::MockDisplay;
using rapid::pr2::Pr2;
using rapid::sound::MockSound;
using rapid::sound::SoundPlay;
using ::testing::Return;

class RobotApiNodeTest : public ::testing::Test {
 public:
  RobotApiNodeTest()
      : nh_(),
        display_(),
        sound_(),
        pr2_(display_, sound_),
        api_(pr2_),
        say_srv_(
            nh_.advertiseService("code_it/api/say", &RobotApi::Say, &api_)),
        ask_mc_srv_(nh_.advertiseService("code_it/api/ask_multiple_choice",
                                         &RobotApi::AskMultipleChoice, &api_)),
        disp_msg_srv_(nh_.advertiseService("code_it/api/display_message",
                                           &RobotApi::DisplayMessage, &api_)),
        is_running_pub_(
            nh_.advertise<std_msgs::Bool>("code_it/is_program_running", 5)),
        is_running_sub_(nh_.subscribe("code_it/is_program_running", 10,
                                      &RobotApi::HandleProgramStopped, &api_)) {
  }

  void SetUp() {
    while (!IsNodeReady()) {
      ros::spinOnce();
    }
  }

 protected:
  bool IsNodeReady() {
    return is_running_pub_.getNumSubscribers() > 0 &&
           is_running_sub_.getNumPublishers() > 0;
  }

  bool WaitForMessage(int timeout) {
    std_msgs::BoolConstPtr msg = ros::topic::waitForMessage<std_msgs::Bool>(
        is_running_sub_.getTopic(), nh_, ros::Duration(timeout));
    if (msg.get() == NULL) {
      return false;
    } else {
      return true;
    }
  }

  ros::NodeHandle nh_;
  MockDisplay display_;
  MockSound sound_;
  Pr2 pr2_;
  RobotApi api_;
  ros::ServiceServer say_srv_;
  ros::ServiceServer ask_mc_srv_;
  ros::ServiceServer disp_msg_srv_;
  ros::Publisher is_running_pub_;
  ros::Subscriber is_running_sub_;
};

TEST_F(RobotApiNodeTest, TestCallsAreWiredUp) {
  EXPECT_CALL(sound_, Say("Hello world!"));
  bool on_time =
      ros::service::waitForService("code_it/api/say", ros::Duration(5));
  ASSERT_EQ(true, on_time);

  code_it::SayRequest req;
  req.text = "Hello world!";
  code_it::SayResponse res;
  bool success = ros::service::call("/code_it/api/say", req, res);
  EXPECT_EQ(true, success);
}

TEST_F(RobotApiNodeTest, TestScreenResetsOnProgramEnd) {
  EXPECT_CALL(display_, ShowDefault()).Times(0);
  std_msgs::Bool msg;
  msg.data = true;
  ros::Publisher is_running_pub =
      nh_.advertise<std_msgs::Bool>("code_it/is_program_running", 10);
  is_running_pub.publish(msg);
  bool success = WaitForMessage(5);

  EXPECT_CALL(display_, ShowDefault()).WillOnce(Return(true));
  msg.data = false;
  is_running_pub.publish(msg);
  success = WaitForMessage(5);
  EXPECT_EQ(true, success);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "robot_api_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
