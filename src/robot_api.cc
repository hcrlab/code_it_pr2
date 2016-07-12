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
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pr2_pbd_interaction/ExecuteActionById.h"
#include "rapid_perception/pr2.h"
#include "rapid_perception/rgbd.h"
#include "rapid_perception/scene.h"
#include "rapid_perception/scene_viz.h"
#include "rapid_pr2/pr2.h"
#include "rapid_ros/publisher.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"

namespace rpe = rapid::perception;
using boost::shared_ptr;
using pr2_pbd_interaction::ExecuteActionById;
using std::string;
using visualization_msgs::Marker;

namespace code_it_pr2 {
RobotApi::RobotApi(
    rapid::pr2::Pr2* robot, const ros::Publisher& error_pub,
    const rapid_ros::Publisher<Marker>& marker_pub,
    const rapid_ros::ServiceClient<ExecuteActionById>& pbd_client)
    : robot_(robot),
      error_pub_(error_pub),
      tf_listener_(),
      marker_pub_(marker_pub),
      scene_(),
      scene_viz_(&marker_pub_),
      scene_has_parsed_(false),
      pbd_client_(pbd_client) {}

bool RobotApi::AskMultipleChoice(code_it_msgs::AskMultipleChoiceRequest& req,
                                 code_it_msgs::AskMultipleChoiceResponse& res) {
  string choice;
  bool success =
      robot_->display()->AskMultipleChoice(req.question, req.choices, &choice);
  res.choice = choice;
  if (!success) {
    res.error = errors::ASK_MC_QUESTION;
  }
  return true;
}

bool RobotApi::DisplayMessage(code_it_msgs::DisplayMessageRequest& req,
                              code_it_msgs::DisplayMessageResponse& res) {
  bool success = robot_->display()->ShowMessage(req.h1_text, req.h2_text);
  if (!success) {
    res.error = errors::DISPLAY_MESSAGE;
  }
  return true;
}

bool RobotApi::FindObjects(code_it_msgs::FindObjectsRequest& req,
                           code_it_msgs::FindObjectsResponse& res) {
  bool success = rpe::pr2::GetManipulationScene(tf_listener_, &scene_);
  if (!success) {
    res.error = errors::GET_SCENE;
    return true;
  }
  scene_viz_.set_scene(scene_);
  scene_viz_.Visualize();
  scene_has_parsed_ = true;
  rpe::HSurface tt = scene_.primary_surface();
  const std::vector<rpe::Object>& objects = tt.objects();
  for (size_t i = 0; i < objects.size(); ++i) {
    const rpe::Object& obj = objects[i];
    code_it_msgs::Object msg;
    msg.name = obj.name();
    msg.pose = obj.pose();
    msg.scale = obj.scale();
    res.objects.push_back(msg);
  }
  return true;
}

bool RobotApi::IsGripperOpen(code_it_msgs::IsGripperOpenRequest& req,
                             code_it_msgs::IsGripperOpenResponse& res) {
  if (req.gripper.id == code_it_msgs::GripperId::RIGHT) {
    res.is_open = robot_->right_gripper()->IsOpen();
  } else if (req.gripper.id == code_it_msgs::GripperId::LEFT) {
    res.is_open = robot_->left_gripper()->IsOpen();
  } else {
    res.error = errors::IS_OPEN_AMBIG;
    return true;
  }
  return true;
}

bool RobotApi::LookAt(code_it_msgs::LookAtRequest& req,
                      code_it_msgs::LookAtResponse& res) {
  bool success = robot_->head()->LookAt(req.target);
  if (!success) {
    res.error = errors::LOOK_AT;
  }
  return true;
}

bool RobotApi::Pick(code_it_msgs::PickRequest& req,
                    code_it_msgs::PickResponse& res) {
  if (!scene_has_parsed_) {
    res.error = errors::PICK_SCENE_NOT_PARSED;
    return true;
  }
  bool has_left_object = robot_->left_gripper()->is_holding_object();
  bool has_right_object = robot_->right_gripper()->is_holding_object();

  // Check if both hands are full
  if (has_left_object && has_right_object) {
    res.error = errors::PICK_ARMS_FULL;
    return true;
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

  rapid::perception::Object object;
  if (!scene_.GetObject(req.object.name, &object)) {
    res.error = errors::PICK_OBJECT_NOT_FOUND;
    return true;
  }
  bool success = false;
  if (arm_id == code_it_msgs::ArmId::LEFT) {
    if (has_left_object) {
      res.error = errors::PICK_LEFT_FULL;
      return true;
    }
    success = robot_->left_picker()->Pick(object);
  } else {
    if (has_right_object) {
      res.error = errors::PICK_RIGHT_FULL;
      return true;
    }
    success = robot_->right_picker()->Pick(object);
  }
  if (!success) {
    res.error = errors::PICK_OBJECT;
    return true;
  }
  return true;
}

bool RobotApi::Place(code_it_msgs::PlaceRequest& req,
                     code_it_msgs::PlaceResponse& res) {
  bool has_left_object = robot_->left_gripper()->is_holding_object();
  bool has_right_object = robot_->right_gripper()->is_holding_object();

  // Check if both hands are empty
  if (!has_left_object && !has_right_object) {
    res.error = errors::PLACE_NO_OBJECTS;
    return true;
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
    res.error = errors::PLACE_NO_LEFT_OBJECT;
    return true;
  }
  if (arm_id == code_it_msgs::ArmId::RIGHT && !has_right_object) {
    res.error = errors::PLACE_NO_RIGHT_OBJECT;
    return true;
  }

  // Get the tabletop.
  rpe::Scene scene;
  bool success = rpe::pr2::GetManipulationScene(tf_listener_, &scene);
  if (!success) {
    res.error = errors::GET_SCENE;
    return true;
  }
  scene_viz_.set_scene(scene_);
  scene_viz_.Visualize();
  const rpe::HSurface& tt = scene.primary_surface();

  if (arm_id == code_it_msgs::ArmId::LEFT) {
    rpe::Object obj;
    robot_->left_gripper()->HeldObject(&obj);
    success = robot_->left_placer()->Place(obj, tt);
  } else {
    rpe::Object obj;
    robot_->right_gripper()->HeldObject(&obj);
    success = robot_->right_placer()->Place(obj, tt);
  }
  if (!success) {
    res.error = errors::PLACE_OBJECT;
    return true;
  }
  return true;
}

bool RobotApi::RunPbdAction(code_it_msgs::RunPbdActionRequest& req,
                            code_it_msgs::RunPbdActionResponse& res) {
  ExecuteActionById::Request pbd_req;
  ExecuteActionById::Response pbd_res;
  pbd_req.action_id = req.action_id;
  bool success = pbd_client_.call(pbd_req, pbd_res);
  if (!success) {
    res.error = errors::PBD_ACTION_FAILED;
    return true;
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
  bool success = true;
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
    res.error = errors::TUCK_DEPLOY;
    return true;
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
