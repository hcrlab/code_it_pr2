#include "code_it_pr2/robot_api.h"

#include "boost/shared_ptr.hpp"
#include "code_it_msgs/Say.h"
#include "code_it_msgs/AskMultipleChoice.h"
#include "code_it_msgs/DisplayMessage.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "rapid_display/display.h"
#include "rapid_pr2/pr2.h"
#include "rapid_sound/sound.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

using ::testing::Return;
using boost::shared_ptr;
using code_it_pr2::RobotApi;
using rapid::display::MockDisplay;
using rapid::manipulation::MockArm;
using rapid::pr2::MockGripper;
using rapid::pr2::MockHead;
using rapid::pr2::Pr2;
using rapid::sound::MockSound;
using rapid::sound::SoundPlay;

class RobotApiNodeTest : public ::testing::Test {
 public:
  RobotApiNodeTest()
      : nh_(),
        left_arm_(),
        right_arm_(),
        display_(),
        left_gripper_(),
        right_gripper_(),
        head_(),
        sound_(),
        pr2_(new Pr2(left_arm_, right_arm_, display_, left_gripper_,
                     right_gripper_, head_, sound_)),
        api_(pr2_),
        say_srv_(
            nh_.advertiseService("code_it/api/say", &RobotApi::Say, &api_)),
        ask_mc_srv_(nh_.advertiseService("code_it/api/ask_multiple_choice",
                                         &RobotApi::AskMultipleChoice, &api_)),
        disp_msg_srv_(nh_.advertiseService("code_it/api/display_message",
                                           &RobotApi::DisplayMessage, &api_)),
        is_running_pub_(
            nh_.advertise<std_msgs::Bool>("code_it/is_program_running", 5)),
        is_running_sub_(
            nh_.subscribe("code_it/is_program_running", 10,
                          &RobotApiNodeTest::HandleProgramRunningMsg, this)),
        run_msg_received_(false) {}

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

  void HandleProgramRunningMsg(const std_msgs::Bool& msg) {
    api_.HandleProgramStopped(msg);
    run_msg_received_ = true;
  }

  ros::NodeHandle nh_;
  MockArm left_arm_;
  MockArm right_arm_;
  MockDisplay display_;
  MockGripper left_gripper_;
  MockGripper right_gripper_;
  MockHead head_;
  MockSound sound_;
  shared_ptr<Pr2> pr2_;
  RobotApi api_;
  ros::ServiceServer say_srv_;
  ros::ServiceServer ask_mc_srv_;
  ros::ServiceServer disp_msg_srv_;
  ros::Publisher is_running_pub_;
  ros::Subscriber is_running_sub_;

  bool run_msg_received_;
};

TEST_F(RobotApiNodeTest, TestCallsAreWiredUp) {
  EXPECT_CALL(sound_, Say("Hello world!"));
  bool on_time =
      ros::service::waitForService("code_it/api/say", ros::Duration(5));
  ASSERT_EQ(true, on_time);

  code_it_msgs::SayRequest req;
  req.text = "Hello world!";
  code_it_msgs::SayResponse res;
  bool success = ros::service::call("/code_it/api/say", req, res);
  EXPECT_EQ(true, success);
}

TEST_F(RobotApiNodeTest, TestScreenResetsOnProgramEnd) {
  EXPECT_CALL(display_, ShowDefault()).WillOnce(Return(true));
  std_msgs::Bool msg;
  msg.data = false;
  is_running_pub_.publish(msg);
  while (!run_msg_received_) {
    ros::spinOnce();
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "robot_api_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
