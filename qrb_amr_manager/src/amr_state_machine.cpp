/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "amr_state_machine.hpp"
#include "unistd.h"

namespace qrb {
namespace amr_manager {

AMRStateMachine::AMRStateMachine() {
  running_ = true;
  current_state_ = IN_ACTIVE;
  thread_handle_msg_ = std::make_shared<std::thread>(
      std::mem_fn(&AMRStateMachine::handle_msg), this);
  send_mapper_cmd_ = false;
  send_navigator_cmd_ = false;
}

AMRStateMachine::~AMRStateMachine() {
  running_ = false;
  queue_.notify();
  if (thread_handle_msg_->joinable()) {
    thread_handle_msg_->join();
  }
}

void AMRStateMachine::register_start_p2p_nav_callback(start_p2p_func_t cb) {
  RCLCPP_INFO(logger_, "register_start_p2p_nav_callback");
  start_p2p_cb_ = cb;
}

void AMRStateMachine::register_start_follow_path_callback(
    start_follow_path_func_t cb) {
  RCLCPP_INFO(logger_, "register_start_follow_path_callback");
  start_follow_path_cb_ = cb;
}

void AMRStateMachine::register_start_waypoint_follow_path_callback(
    start_waypoint_follow_path_func_t cb) {
  RCLCPP_INFO(logger_, "register_start_waypoint_follow_path_callback");
  start_waypoint_follow_path_cb_ = cb;
}

void AMRStateMachine::register_sub_cmd_callback(sub_cmd_func_t cb) {
  RCLCPP_INFO(logger_, "register_sub_cmd_callback");
  sub_cmd_cb_ = cb;
}

void AMRStateMachine::register_start_charging_callback(
    start_charging_func_t cb) {
  RCLCPP_INFO(logger_, "register_start_charging_callback");
  start_charging_cb_ = cb;
}

void AMRStateMachine::register_notify_exception_callback(
    notify_exception_func_t cb) {
  RCLCPP_INFO(logger_, "register_notify_exception_callback");
  notify_exception_cb_ = cb;
}

void AMRStateMachine::register_send_amr_state_changed_callback(
    send_amr_state_changed_func_t cb) {
  RCLCPP_INFO(logger_, "register_send_amr_state_changed_callback");
  send_amr_state_changed_cb_ = cb;
}

void AMRStateMachine::register_navigate_to_charging_callback(
    navigate_to_charging_func_t cb) {
  RCLCPP_INFO(logger_, "register_navigate_to_charging_callback");
  navigate_to_charging_cb_ = cb;
}

void AMRStateMachine::register_slam_command_callback(slam_command_func_t cb) {
  RCLCPP_INFO(logger_, "register_slam_command_callback");
  slam_command_cb_ = cb;
}

void AMRStateMachine::register_publish_twist_callback(publish_twist_func_t cb) {
  publish_twist_cb_ = cb;
}

void AMRStateMachine::init_amr() {
  Message msg;
  msg.type = Message::INIT_AMR;
  queue_.push(msg);
}

void AMRStateMachine::release_amr() {
  Message msg;
  msg.type = Message::RELEASE_AMR;
  queue_.push(msg);
}

bool AMRStateMachine::load_map() {
  bool result;
  slam_command_cb_((uint8_t)Slam_Command::LoadMap, result);
  if (!result) {
    RCLCPP_INFO(logger_, "Load map failed");
    return false;
  }

  send_me_finish_message();
  return true;
}

bool AMRStateMachine::start_mapping() {
  bool result;
  slam_command_cb_((uint8_t)Slam_Command::StartMapping, result);
  if (!result) {
    RCLCPP_INFO(logger_, "Start mapping failed");
    return false;
  }

  send_me_message();
  return true;
}

bool AMRStateMachine::stop_mapping() {
  bool result;
  slam_command_cb_((uint8_t)Slam_Command::StopMapping, result);
  if (!result) {
    RCLCPP_INFO(logger_, "Stop mapping failed");
    return false;
  }

  send_me_completed_message();
  return true;
}

void AMRStateMachine::process_cmd(int cmd, void *buffer, size_t len) {
  Message msg;
  msg.type = cmd;
  msg.param = buffer;
  msg.len = len;
  queue_.push(msg);
}

void AMRStateMachine::process_cmd(int cmd, uint32_t goal_id,
                                  vector<uint32_t> &ids) {
  Message msg;
  msg.type = cmd;
  msg.goal_id = goal_id;
  msg.ids.assign(ids.begin(), ids.end());
  queue_.push(msg);
}

void AMRStateMachine::process_sub_cmd(int message) {
  Message msg;
  msg.type = message;
  queue_.push(msg);
}

void AMRStateMachine::process_event(int event, uint8_t error_code) {
  Message msg;
  msg.type = event;
  msg.error_code = error_code;
  queue_.push(msg);
}

void AMRStateMachine::process_event(int event) {
  Message msg;
  msg.type = event;
  queue_.push(msg);
}

void AMRStateMachine::handle_msg() {
  Message msg;
  bool execute = true;
  while (running_) {
    queue_.wait(msg);
    execute = handle_message(msg);
    if (!execute) {
      handle_failed(msg);
      continue;
    }
  }
}

bool AMRStateMachine::handle_message(const Message &msg) {
  RCLCPP_INFO(logger_, "Receive message: %s",
              Message::msg_to_string(msg.type).c_str());
  int last_state = current_state_;
  switch (msg.type) {
  case Message::INIT_AMR:
    if (current_state_ == AMRStateMachine::IN_ACTIVE) {
      if (!has_map_) {
        update_state(last_state, AMRStateMachine::IDLE);
      } else {
        update_state(last_state, AMRStateMachine::READY);
      }
    }
    break;
  case Message::RELEASE_AMR:
    if (current_state_ == AMRStateMachine::ON_AE) {
      // TODO:
    } else if (current_state_ == AMRStateMachine::ON_P2PNAV) {
      sub_cmd_cb_(true, SubCommand::CANCEL);
    } else if (current_state_ == AMRStateMachine::ON_FOLLOW_PATH) {
      sub_cmd_cb_(false, SubCommand::CANCEL);
    }
    update_state(last_state, AMRStateMachine::IN_ACTIVE);
    break;
  case Message::ME_COMPLETED:
    if (current_state_ != AMRStateMachine::ON_ME) {
      return false;
    }
    update_state(last_state, AMRStateMachine::ME_DONE);
    enter_me_done_state();
    break;
  case Message::ME:
    if (current_state_ != AMRStateMachine::IDLE &&
        current_state_ != AMRStateMachine::READY) {
      return false;
    }
    update_state(last_state, AMRStateMachine::ON_ME);
    enter_on_me_state();
    break;
  case Message::AE:
    if (current_state_ != AMRStateMachine::IDLE &&
        current_state_ != AMRStateMachine::READY) {
      return false;
    }
    update_state(last_state, AMRStateMachine::ON_AE);
    enter_on_ae_state();
    break;
  case Message::P2PNAV:
    if (current_state_ != AMRStateMachine::READY &&
        current_state_ != AMRStateMachine::FOLLOW_PATH_WAIT &&
        current_state_ != AMRStateMachine::P2PNAV_WAIT) {
      return false;
    }
    if (current_state_ == AMRStateMachine::FOLLOW_PATH_WAIT) {
      RCLCPP_INFO(logger_, "Cancel last follow path");
      sub_cmd_cb_(false, SubCommand::CANCEL);
      usleep(2 * 1000 * 1000);
    }

    update_state(last_state, AMRStateMachine::ON_P2PNAV);
    enter_on_p2p_state(msg);
    break;
  case Message::FOLLOW_PATH:
    if (current_state_ != AMRStateMachine::READY &&
        current_state_ != AMRStateMachine::P2PNAV_WAIT &&
        current_state_ != AMRStateMachine::FOLLOW_PATH_WAIT) {
      return false;
    }
    if (current_state_ == AMRStateMachine::P2PNAV_WAIT) {
      RCLCPP_INFO(logger_, "Cancel last p2p navigation");
      sub_cmd_cb_(true, SubCommand::CANCEL);
      usleep(2 * 1000 * 1000);
    }

    update_state(last_state, AMRStateMachine::ON_FOLLOW_PATH);
    enter_on_follow_path_state(msg);
    break;
  case Message::WAYPOINT_FOLLOW_PATH:
    if (current_state_ != AMRStateMachine::READY &&
        current_state_ != AMRStateMachine::P2PNAV_WAIT &&
        current_state_ != AMRStateMachine::FOLLOW_PATH_WAIT) {
      return false;
    }
    update_state(last_state, AMRStateMachine::ON_FOLLOW_PATH);
    enter_on_waypoint_follow_path_state(msg);
    break;
  case Message::AE_FINISH:
    if (current_state_ == AMRStateMachine::ON_AE ||
        current_state_ == AMRStateMachine::IDLE) {
      update_state(last_state, AMRStateMachine::LOCALIZATION);
      has_map_ = true;
    }
    break;
  case Message::ME_FINISH:
    if ((current_state_ == AMRStateMachine::IDLE) ||
        (current_state_ == AMRStateMachine::ME_DONE)) {
      update_state(last_state, AMRStateMachine::LOCALIZATION);
      has_map_ = true;
      enter_localization_state();
    }
    break;
  case Message::RELOCALIZATION_PASS:
    if (current_state_ == AMRStateMachine::LOCALIZATION) {
      update_state(last_state, AMRStateMachine::READY);
    }
    break;
  case Message::P2PNAV_FINISH:
    if (current_state_ == AMRStateMachine::ON_P2PNAV) {
      update_state(last_state, AMRStateMachine::READY);
    }
    break;
  case Message::FOLLOW_PATH_FINISH:
    if (current_state_ == AMRStateMachine::ON_FOLLOW_PATH) {
      update_state(last_state, AMRStateMachine::READY);
    }
    break;
  case Message::LOW_POWER:
    if (current_state_ == AMRStateMachine::ON_ME) {
      if (!has_map_) {
        update_state(last_state, AMRStateMachine::IDLE);
        enter_idle_state();
      } else {
        enter_ready_state();
        update_state(last_state, AMRStateMachine::READY);
      }
    } else if (current_state_ == AMRStateMachine::READY ||
               current_state_ == AMRStateMachine::ON_FOLLOW_PATH ||
               current_state_ == AMRStateMachine::FOLLOW_PATH_WAIT ||
               current_state_ == AMRStateMachine::ON_P2PNAV ||
               current_state_ == AMRStateMachine::P2PNAV_WAIT) {
      update_state(last_state, AMRStateMachine::READY);
      enter_ready_state();
      send_return_charging_message();
    }
    break;
  case Message::NORMAL_POWER:
    if (current_state_ == AMRStateMachine::LOW_POWER_CHARGING) {
      update_state(last_state, AMRStateMachine::READY);
      enter_ready_state();
    }
    break;
  case Message::AMR_EXCEPTION:
  {
    enter_on_error_state();
    update_state(last_state, AMRStateMachine::ON_ERROR);
    uint8_t error_code = msg.error_code;
    notify_exception_cb_(true, error_code);
  } break;
  case Message::AMR_NORMAL:
    if (current_state_ == AMRStateMachine::ON_ERROR) {
      update_state(last_state, AMRStateMachine::READY);
      enter_ready_state();
      notify_exception_cb_(false, 0);
    }
    break;
  case Message::CANCEL:
    if (current_state_ == AMRStateMachine::ON_ME) {
      if (!has_map_) {
        update_state(last_state, AMRStateMachine::IDLE);
        enter_idle_state();
      } else {
        enter_ready_state();
        update_state(last_state, AMRStateMachine::READY);
      }
    } else if (current_state_ == AMRStateMachine::ON_FOLLOW_PATH ||
               current_state_ == AMRStateMachine::ON_P2PNAV ||
               current_state_ == AMRStateMachine::P2PNAV_WAIT ||
               current_state_ == AMRStateMachine::FOLLOW_PATH_WAIT ||
               current_state_ == AMRStateMachine::ON_RETURN_CHARGING) {
      enter_ready_state();
      update_state(last_state, AMRStateMachine::READY);
    } else {
      return false;
    }
    break;
  case Message::PAUSE:
    RCLCPP_INFO(logger_, "Message::PAUSE:,current_state_=%d", current_state_);
    if (current_state_ == AMRStateMachine::ON_FOLLOW_PATH) {
      update_state(last_state, AMRStateMachine::FOLLOW_PATH_WAIT);
      enter_follow_path_wait_state();
    } else if (current_state_ == AMRStateMachine::ON_P2PNAV) {
      update_state(last_state, AMRStateMachine::P2PNAV_WAIT);
      enter_p2p_wait_state();
    } else {
      return false;
    }
    break;
  case Message::RESUME:
    if (current_state_ == AMRStateMachine::FOLLOW_PATH_WAIT) {
      update_state(last_state, AMRStateMachine::ON_FOLLOW_PATH);
      sub_cmd_cb_(false, SubCommand::RESUME);
    } else if (current_state_ == AMRStateMachine::P2PNAV_WAIT) {
      update_state(last_state, AMRStateMachine::ON_P2PNAV);
      sub_cmd_cb_(true, SubCommand::RESUME);
    } else {
      return false;
    }
    break;
  case Message::RETURN_CHARGING_FINISH:
    if (current_state_ == AMRStateMachine::ON_RETURN_CHARGING) {
      update_state(last_state, AMRStateMachine::LOW_POWER_CHARGING);
      enter_low_power_charging_state();
      destory_return_charging_station_thread();
    } else {
      return false;
    }
    break;
  case Message::RETURN_CHARGING:
    if (current_state_ == AMRStateMachine::READY) {
      update_state(last_state, AMRStateMachine::ON_RETURN_CHARGING);
      enter_on_return_charging_state();
    } else {
      return false;
    }
    break;
  default:
    break;
  }
  RCLCPP_INFO(logger_, "Finish handle_message");
  return true;
}

void AMRStateMachine::notify_state_machine_changed() {
  RCLCPP_INFO(logger_, "send_amr_state_changed:%s",
              get_current_state().c_str());
  if (send_amr_state_changed_cb_ != nullptr) {
    send_amr_state_changed_cb_(current_state_);
  } else {
    RCLCPP_ERROR(logger_, "send_amr_state_changed_cb_ is null");
  }
}

void AMRStateMachine::handle_failed(const Message &msg) {
  // TODO:
}

std::string AMRStateMachine::get_current_state() {
  return state_to_string(current_state_);
}

void AMRStateMachine::enter_idle_state() {
  if (current_state_ == AMRStateMachine::ON_AE) {
    // TODO:
    return;
  }
}

void AMRStateMachine::enter_ready_state() {
  if (send_navigator_cmd_) {
    if ((current_state_ == AMRStateMachine::FOLLOW_PATH_WAIT) ||
        (current_state_ == AMRStateMachine::ON_FOLLOW_PATH)) {
      RCLCPP_INFO(logger_, "Cancel follow path");
      sub_cmd_cb_(false, SubCommand::CANCEL);
    } else if ((current_state_ == AMRStateMachine::ON_P2PNAV) ||
               (current_state_ == AMRStateMachine::P2PNAV_WAIT)) {
      RCLCPP_INFO(logger_, "Cancel p2p navigaiton");
      sub_cmd_cb_(true, SubCommand::CANCEL);
    } else if (current_state_ == AMRStateMachine::ON_RETURN_CHARGING) {
      RCLCPP_INFO(logger_, "Cancel return charging station");
      sub_cmd_cb_(true, SubCommand::CANCEL);
      destory_return_charging_station_thread();
    }
    send_navigator_cmd_ = false;
  } else if (send_mapper_cmd_) {
    RCLCPP_INFO(logger_, "Cancel auto mapping navigaiton");
    // TODO:
    send_mapper_cmd_ = false;
  }
}

void AMRStateMachine::enter_me_done_state() {
  send_mapper_cmd_ = true;
  save_map();
  send_me_finish_message();
}

void AMRStateMachine::enter_on_me_state() { send_mapper_cmd_ = true; }

void AMRStateMachine::enter_on_ae_state() {
  send_mapper_cmd_ = true;
  // TODO:
}

void AMRStateMachine::enter_on_p2p_state(const Message &msg) {
  send_navigator_cmd_ = true;
  start_charging_cb_(false);
  start_p2p_cb_(msg.param);
}

void AMRStateMachine::enter_on_follow_path_state(const Message &msg) {
  send_navigator_cmd_ = true;
  path_buffer_ = msg.param;
  start_charging_cb_(false);
  start_follow_path_cb_(path_buffer_);
}

void AMRStateMachine::enter_on_waypoint_follow_path_state(const Message &msg) {
  send_navigator_cmd_ = true;
  uint32_t goal_id = msg.goal_id;
  vector<uint32_t> ids;
  ids.assign(msg.ids.begin(), msg.ids.end());
  start_charging_cb_(false);
  start_waypoint_follow_path_cb_(goal_id, ids);
}

void AMRStateMachine::enter_on_error_state() {
  if (send_navigator_cmd_) {
    RCLCPP_INFO(logger_, "Send cancel nav command when amr is error, state = %d", current_state_);
    if (current_state_ == AMRStateMachine::FOLLOW_PATH_WAIT ||
        current_state_ == AMRStateMachine::ON_FOLLOW_PATH) {
      sub_cmd_cb_(false, SubCommand::CANCEL);
    } else if (current_state_ == AMRStateMachine::ON_P2PNAV ||
               current_state_ == AMRStateMachine::P2PNAV_WAIT) {
      sub_cmd_cb_(true, SubCommand::CANCEL);
    } else {
      RCLCPP_INFO(logger_, "Check the nav state when amr is error");
    }
    send_navigator_cmd_ = false;
  } else if (send_mapper_cmd_) {
    // TODO:
    RCLCPP_INFO(logger_, "Send cancel mapping command when amr is error");
    send_mapper_cmd_ = false;
  } else {
    RCLCPP_INFO(logger_, "Nothing to do when amr is error");
  }
}

void AMRStateMachine::enter_follow_path_wait_state() {
  sub_cmd_cb_(false, SubCommand::PAUSE);
}

void AMRStateMachine::enter_on_return_charging_state() {
  send_navigator_cmd_ = true;
  create_return_charging_station_thread();
}

void AMRStateMachine::create_return_charging_station_thread() {
  // create sub thread
  RCLCPP_INFO(logger_, "Create return charging station thread");
  auto fun = [this]() -> void {
    pthread_setname_np(pthread_self(), "return_charging");
    return_charging_station_function(this);
  };
  return_charging_station_thread_ = std::make_shared<std::thread>(fun);
  RCLCPP_INFO(logger_, "Create return charging station thread successfully");
}

void AMRStateMachine::return_charging_station_function(void *arg) {
  AMRStateMachine *p = (AMRStateMachine *)arg;
  RCLCPP_INFO(p->logger_, "return_charging_station_function");
  return_charging_station();
  start_charging_cb_(true);
}

void AMRStateMachine::destory_return_charging_station_thread() {
  RCLCPP_INFO(logger_, "destory_return_charging_station_thread");
  if (return_charging_station_thread_ && return_charging_station_thread_->joinable()) {
    return_charging_station_thread_->join();
    RCLCPP_INFO(logger_, "return charging station thread end");
    return_charging_station_thread_ = nullptr;
  }
  RCLCPP_INFO(logger_, "finish destory return charging station thread");
}

void AMRStateMachine::enter_p2p_wait_state() {
  // cancel goal, start goal again when resume
  sub_cmd_cb_(true, SubCommand::PAUSE);
}

void AMRStateMachine::enter_low_power_charging_state() {
  // TODO:
}

void *AMRStateMachine::get_current_path() { return path_buffer_; }

void AMRStateMachine::return_charging_station() { navigate_to_charging_cb_(); }

bool AMRStateMachine::check_potential_state(int cmd) {
  RCLCPP_INFO(logger_, "Receive cmd: %s, current_state:%s",
              Command::cmd_to_string(cmd).c_str(), get_current_state().c_str());
  if (current_state_ == ON_ERROR) {
    RCLCPP_ERROR(logger_, "check_potential_state(%d,%d) return false", cmd,
              current_state_);
    return false;
  }

  if (cmd == Command::OTHER) {
    RCLCPP_INFO(logger_, "check_potential_state(%d,%d) return true", cmd,
                current_state_);
    return true;
  }
  if (((cmd == Command::AE) || (cmd == Command::ME)) &&
      ((current_state_ == IDLE) || (current_state_ == READY))) {
    RCLCPP_INFO(logger_, "check_potential_state(%d,%d) return true", cmd,
                current_state_);
    return true;
  }
  if ((cmd == Command::P2PNAV) &&
      ((current_state_ == READY) || (current_state_ == FOLLOW_PATH_WAIT) ||
       (current_state_ == P2PNAV_WAIT))) {
    RCLCPP_INFO(logger_, "check_potential_state(%d,%d) return true", cmd,
                current_state_);
    return true;
  }
  if (((cmd == Command::FOLLOW_PATH) ||
       (cmd == Command::WAYPOINT_FOLLOW_PATH)) &&
      ((current_state_ == READY) || (current_state_ == FOLLOW_PATH_WAIT) ||
       (current_state_ == P2PNAV_WAIT))) {
    RCLCPP_INFO(logger_, "check_potential_state(%d,%d) return true", cmd,
                current_state_);
    return true;
  }
  if ((cmd == Command::CHARGING) &&
      ((current_state_ == IDLE) || (current_state_ == READY))) {
    RCLCPP_INFO(logger_, "check_potential_state(%d,%d) return true", cmd,
                current_state_);
    return true;
  }
  if ((cmd == Command::SUB_CMD) &&
      ((current_state_ == ON_P2PNAV) ||
       (current_state_ == ON_FOLLOW_PATH) ||
       (current_state_ == P2PNAV_WAIT) ||
       (current_state_ == FOLLOW_PATH_WAIT) ||
       (current_state_ == ON_RETURN_CHARGING) ||
       (current_state_ == ON_ME))) {
    RCLCPP_INFO(logger_, "check_potential_state(%d,%d) return true", cmd,
                current_state_);
    return true;
  }

  RCLCPP_INFO(logger_, "check_potential_state(%d,%d) return false", cmd,
              current_state_);
  return false;
}

void AMRStateMachine::save_map() {
  if (slam_command_cb_ == nullptr) {
    RCLCPP_ERROR(logger_, "slam_command_cb_ is nullptr");
    return;
  }

  bool result;
  slam_command_cb_((uint8_t)Slam_Command::SaveMap, result);
  if (!result) {
    RCLCPP_ERROR(logger_, "Save map failed");
  }
}

void AMRStateMachine::enter_localization_state() {
  bool result = enter_localization_mode();
  if (!result) {
    RCLCPP_ERROR(logger_, "Enter localization mode failed");
    return;
  }

  result = get_relocalization_state();
  if (result) {
    RCLCPP_INFO(logger_, "Current localization is ready");
    send_relocalization_pass_message();
    return;
  }

  start_rotation();

  while (true) {
    usleep(1000 * 1000); // sleep 1s
    RCLCPP_INFO(logger_, "Wait 1 second");
    result = get_relocalization_state();
    if (result) {
      RCLCPP_INFO(logger_, "Current localization is ready");
      stop_rotation();
      send_relocalization_pass_message();
      return;
    }
  }
}

bool AMRStateMachine::enter_localization_mode() {
  if (slam_command_cb_ == nullptr) {
    RCLCPP_ERROR(logger_, "slam_command_cb_ is nullptr");
    return false;
  }

  bool result;
  slam_command_cb_((uint8_t)Slam_Command::StartLocalization, result);
  if (result) {
    RCLCPP_INFO(logger_, "Enter localization mode success");
  } else {
    RCLCPP_ERROR(logger_, "Enter localization mode failed");
  }
  return result;
}

bool AMRStateMachine::get_relocalization_state() {
  if (slam_command_cb_ == nullptr) {
    RCLCPP_ERROR(logger_, "slam_command_cb_ is nullptr");
    return false;
  }

  bool result;
  slam_command_cb_((uint8_t)Slam_Command::Relocalization, result);
  if (result) {
    RCLCPP_INFO(logger_, "relocalization is pass");
  } else {
    RCLCPP_ERROR(logger_, "relocalization is failed");
  }
  return result;
}

void AMRStateMachine::start_rotation() {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = ANGULAR_VELOCITY;
  if (publish_twist_cb_ != nullptr) {
    publish_twist_cb_(twist);
  } else {
    RCLCPP_ERROR(logger_, "publish_twist_cb_ is nullptr");
  }
}

void AMRStateMachine::stop_rotation() {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = 0;
  if (publish_twist_cb_ != nullptr) {
    publish_twist_cb_(twist);
  } else {
    RCLCPP_ERROR(logger_, "publish_twist_cb_ is nullptr");
  }
}

void AMRStateMachine::send_me_finish_message() {
  Message msg;
  msg.type = Message::ME_FINISH;
  queue_.push(msg);
}

void AMRStateMachine::send_me_message() {
  Message msg;
  msg.type = Message::ME;
  queue_.push(msg);
}

void AMRStateMachine::send_me_completed_message() {
  Message msg;
  msg.type = Message::ME_COMPLETED;
  queue_.push(msg);
}

void AMRStateMachine::send_relocalization_pass_message() {
  Message msg;
  msg.type = Message::RELOCALIZATION_PASS;
  queue_.push(msg);
}

void AMRStateMachine::send_return_charging_message() {
  Message msg;
  msg.type = Message::RETURN_CHARGING;
  queue_.push(msg);
}

void AMRStateMachine::update_state(int last_state, int state) {
  current_state_ = state;
  if (last_state != current_state_) {
    notify_state_machine_changed();
  }
  RCLCPP_INFO(logger_, "current state: %s, last state: %s",
              get_current_state().c_str(), state_to_string(last_state).c_str());
}
} // namespace amr_manager
} // namespace qrb