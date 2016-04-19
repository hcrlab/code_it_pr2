#include "code_it_pr2/robot_api.h"

#include <string>
#include <vector>

#include "boost/shared_ptr.hpp"
#include "code_it_msgs/ArmId.h"
#include "code_it_msgs/AskMultipleChoice.h"
#include "code_it_msgs/DisplayMessage.h"
#include "code_it_msgs/GripperId.h"
#include "code_it_msgs/LookAt.h"
#include "code_it_msgs/Pick.h"
#include "code_it_msgs/Place.h"
#include "code_it_msgs/Say.h"
#include "code_it_msgs/SetGripper.h"
#include "code_it_msgs/TuckArms.h"
#include "rapid_perception/pr2.h"
#include "rapid_perception/rgbd.hpp"
#include "rapid_perception/scene.h"
#include "rapid_pr2/pr2.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"

namespace rpe = rapid::perception;
using boost::shared_ptr;
using std::string;

namespace code_it_pr2 {
RobotApi::RobotApi(rapid::pr2::Pr2* robot, const ros::Publisher& error_pub)
    : robot_(robot),
      error_pub_(error_pub),
      tf_listener_(),
      scene_(),
      scene_has_parsed_(false) {}

bool RobotApi::AskMultipleChoice(code_it_msgs::AskMultipleChoiceRequest& req,
                                 code_it_msgs::AskMultipleChoiceResponse& res) {
  string choice;
  bool success =
      robot_->display()->AskMultipleChoice(req.question, req.choices, &choice);
  res.choice = choice;
  if (!success) {
    PublishError(errors::ASK_MC_QUESTION);
  }
  return success;
}

bool RobotApi::DisplayMessage(code_it_msgs::DisplayMessageRequest& req,
                              code_it_msgs::DisplayMessageResponse& res) {
  return robot_->display()->ShowMessage(req.h1_text, req.h2_text);
}

bool RobotApi::FindObjects(code_it_msgs::FindObjectsRequest& req,
                           code_it_msgs::FindObjectsResponse& res) {
  bool success = rpe::pr2::GetManipulationScene(tf_listener_, &scene_);
  if (!success) {
    PublishError(errors::GET_SCENE);
    return false;
  }
  scene_has_parsed_ = true;
  boost::shared_ptr<rpe::Tabletop> tt = scene_.GetPrimarySurface();
  const std::vector<rpe::Object>* objects = tt->objects();
  for (size_t i = 0; i < objects->size(); ++i) {
    const rpe::Object& obj = (*objects)[i];
    code_it_msgs::Object msg;
    msg.name = obj.name();
    msg.pose = obj.pose();
    msg.scale = obj.scale();
    res.objects.push_back(msg);
  }
  return true;
}

bool RobotApi::LookAt(code_it_msgs::LookAtRequest& req,
                      code_it_msgs::LookAtResponse& res) {
  return robot_->head()->LookAt(req.target);
}

bool RobotApi::Pick(code_it_msgs::PickRequest& req,
                    code_it_msgs::PickResponse& res) {
  if (!scene_has_parsed_) {
    PublishError(errors::PICK_SCENE_NOT_PARSED);
    return false;
  }
  bool has_left_object = robot_->left_gripper()->is_holding_object();
  bool has_right_object = robot_->right_gripper()->is_holding_object();

  // Check if both hands are full
  if (has_left_object && has_right_object) {
    PublishError(errors::PICK_ARMS_FULL);
    return false;
  }

  // If using the default arm, use whichever arm is free, or the right arm if
  // both arms are free.
  int8_t arm_id = req.arm.arm_id;
  if (arm_id == code_it_msgs::ArmId::DEFAULT) {
    if (has_left_object && !has_right_object) {
      arm_id = code_it_msgs::ArmId::RIGHT;
    } else if (!has_left_object && has_right_object) {
      arm_id = code_it_msgs::ArmId::LEFT;
    } else {
      arm_id = code_it_msgs::ArmId::RIGHT;
    }
  }

  geometry_msgs::PoseStamped ps = req.object.pose;
  geometry_msgs::Vector3 scale = req.object.scale;
  rapid::perception::ScenePrimitive primitive(ps, scale, "object");
  rapid::perception::Object* object;
  scene_.GetObject(req.object.name, &object);
  bool success = false;
  if (arm_id == code_it_msgs::ArmId::LEFT) {
    if (has_left_object) {
      PublishError(errors::PICK_LEFT_FULL);
      return false;
    }
    success = robot_->left_picker()->Pick(*object);
    if (success) {
      robot_->left_gripper()->set_held_object(primitive);
    }
  } else {
    if (has_right_object) {
      PublishError(errors::PICK_RIGHT_FULL);
      return false;
    }
    success = robot_->right_picker()->Pick(*object);
    if (success) {
      robot_->right_gripper()->set_held_object(primitive);
    }
  }
  if (!success) {
    PublishError(errors::PICK_OBJECT);
    return false;
  }
  return true;
}

bool RobotApi::Place(code_it_msgs::PlaceRequest& req,
                     code_it_msgs::PlaceResponse& res) {
  bool has_left_object = robot_->left_gripper()->is_holding_object();
  bool has_right_object = robot_->right_gripper()->is_holding_object();

  // Check if both hands are empty
  if (!has_left_object && !has_right_object) {
    PublishError(errors::PLACE_NO_OBJECTS);
    return false;
  }

  // If using default arm, default to whichever arm has an object, or the right
  // arm if both arms have objects.
  int8_t arm_id = req.arm.arm_id;
  if (arm_id == code_it_msgs::ArmId::DEFAULT) {
    if (has_left_object && !has_right_object) {
      arm_id = code_it_msgs::ArmId::LEFT;
    } else {
      arm_id = code_it_msgs::ArmId::RIGHT;
    }
  }
  if (arm_id == code_it_msgs::ArmId::LEFT && !has_left_object) {
    PublishError(errors::PLACE_NO_LEFT_OBJECT);
    return false;
  }
  if (arm_id == code_it_msgs::ArmId::RIGHT && !has_right_object) {
    PublishError(errors::PLACE_NO_RIGHT_OBJECT);
    return false;
  }

  // Get the tabletop.
  rpe::Scene scene;
  bool success = rpe::pr2::GetManipulationScene(tf_listener_, &scene);
  if (!success) {
    PublishError(errors::GET_SCENE);
    return false;
  }
  boost::shared_ptr<rpe::Tabletop> tt = scene.GetPrimarySurface();

  if (arm_id == code_it_msgs::ArmId::LEFT) {
    rpe::ScenePrimitive obj;
    robot_->left_gripper()->HeldObject(&obj);
    success = robot_->left_placer()->Place(obj, *tt);
    if (success) {
      robot_->left_gripper()->set_is_holding_object(false);
    }
  } else {
    rpe::ScenePrimitive obj;
    robot_->right_gripper()->HeldObject(&obj);
    success = robot_->right_placer()->Place(obj, *tt);
    if (success) {
      robot_->right_gripper()->set_is_holding_object(false);
    }
  }
  if (!success) {
    PublishError(errors::PLACE_OBJECT);
    return false;
  }
  return true;
}

bool RobotApi::Say(code_it_msgs::SayRequest& req,
                   code_it_msgs::SayResponse& res) {
  robot_->sound()->Say(req.text);
  return true;
}

bool RobotApi::SetGripper(code_it_msgs::SetGripperRequest& req,
                          code_it_msgs::SetGripperResponse& res) {
  if (req.gripper.id == code_it_msgs::GripperId::LEFT) {
    if (req.action == code_it_msgs::SetGripperRequest::OPEN) {
      robot_->left_gripper()->Open(req.max_effort);
    } else if (req.action == code_it_msgs::SetGripperRequest::CLOSE) {
      robot_->left_gripper()->Close(req.max_effort);
    }
  } else {
    if (req.action == code_it_msgs::SetGripperRequest::OPEN) {
      robot_->right_gripper()->Open(req.max_effort);
    } else if (req.action == code_it_msgs::SetGripperRequest::CLOSE) {
      robot_->right_gripper()->Close(req.max_effort);
    }
  }
  return true;
}

bool RobotApi::TuckArms(code_it_msgs::TuckArmsRequest& req,
                        code_it_msgs::TuckArmsResponse& res) {
  bool success = false;
  if (req.tuck_left && req.tuck_right) {
    success = robot_->tuck_arms()->TuckArms();
  } else if (req.tuck_left && !req.tuck_right) {
    success = robot_->tuck_arms()->DeployRight();
  } else if (!req.tuck_left && req.tuck_right) {
    success = robot_->tuck_arms()->DeployLeft();
  } else {
    success = robot_->tuck_arms()->DeployArms();
  }
  if (!success) {
    PublishError(errors::TUCK_DEPLOY);
    return false;
  }
  return true;
}

void RobotApi::HandleProgramStopped(const std_msgs::Bool& msg) {
  if (msg.data) {
    return;  // Program is running, nothing to do.
  }
  bool success = robot_->display()->ShowDefault();
  if (!success) {
    PublishError(errors::RESET_SCREEN_ON_STOP);
  }
}

void RobotApi::PublishError(const string& error) {
  std_msgs::String error_msg;
  error_msg.data = error;
  error_pub_.publish(error_msg);
  ROS_ERROR("%s", error.c_str());
}
}  // namespace code_it_pr2
