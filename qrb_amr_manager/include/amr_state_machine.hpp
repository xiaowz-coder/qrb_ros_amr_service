/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef AMR_STATE_MACHINE_HPP_
#define AMR_STATE_MACHINE_HPP_

#include "message_queue.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace std;

#define ANGULAR_VELOCITY 0.19 // the min angular velocity for arm

namespace qrb {
namespace amr_manager {

typedef std::function<void(void *buffer)> start_p2p_func_t;
typedef std::function<void(void *path)> start_follow_path_func_t;
typedef std::function<void(uint8_t goal, vector<uint32_t> &ids)>
    start_waypoint_follow_path_func_t;
typedef std::function<void(bool p2p, uint8_t sub_cmd)> sub_cmd_func_t;
typedef std::function<void(bool start)> start_charging_func_t;
typedef std::function<void(bool exception, int error_code)>
    notify_exception_func_t;
typedef std::function<void(int state)> send_amr_state_changed_func_t;
typedef std::function<void(void)> navigate_to_charging_func_t;
typedef std::function<void(uint8_t mode)> change_mode_func_t;
typedef std::function<void(uint8_t cmd, bool &result)> slam_command_func_t;
typedef std::function<void(geometry_msgs::msg::Twist &velocity)>
    publish_twist_func_t;

enum class Slam_Command {
  StartMapping = 1,
  StopMapping = 2,
  SaveMap = 3,
  LoadMap = 4,
  StartLocalization = 5,
  Relocalization = 6,
};

class Command {
public:
  const static int ME = 1;
  const static int AE = 2;
  const static int P2PNAV = 3;
  const static int FOLLOW_PATH = 4;
  const static int CHARGING = 5;
  const static int WAYPOINT_FOLLOW_PATH = 6;
  const static int SUB_CMD = 7;
  const static int OTHER = 8;

  static std::string cmd_to_string(int cmd) {
    std::string message;
    switch (cmd) {
      case ME:
        message = "ME";
        break;
      case AE:
        message = "AE";
        break;
      case P2PNAV:
        message = "P2PNAV";
        break;
      case FOLLOW_PATH:
        message = "FOLLOW_PATH";
        break;
      case CHARGING:
        message = "CHARGING";
        break;
      case WAYPOINT_FOLLOW_PATH:
        message = "WAYPOINT_FOLLOW_PATH";
        break;
      case SUB_CMD:
        message = "SUB_CMD";
        break;
      case OTHER:
        message = "OTHER";
        break;
      default:
        message = "INVAILD";
        break;
    }
    return message;
  }
};

class SubCommand {
public:
  const static int CANCEL = 1;
  const static int PAUSE = 2;
  const static int RESUME = 3;

  static std::string to_string(int cmd) {
    std::string message;
    switch (cmd) {
    case CANCEL:
      message = "CANCEL";
      break;
    case PAUSE:
      message = "PAUSE";
      break;
    case RESUME:
      message = "RESUME";
      break;
    default:
      message = "INVAILD";
      break;
    }
    return message;
  }
};

class AMRStateMachine {
private:
  std::mutex mtx_;
  std::condition_variable *cv_;
  bool *sub_cmd_success_;
  void *path_buffer_;
  size_t path_len_;
  bool send_mapper_cmd_;
  bool send_navigator_cmd_;
  bool has_map_;
  int current_state_;
  bool running_;

  MessageQueue queue_;
  std::shared_ptr<std::thread> thread_handle_msg_;
  start_p2p_func_t start_p2p_cb_;
  start_follow_path_func_t start_follow_path_cb_;
  start_waypoint_follow_path_func_t start_waypoint_follow_path_cb_;
  sub_cmd_func_t sub_cmd_cb_;
  start_charging_func_t start_charging_cb_;
  notify_exception_func_t notify_exception_cb_;
  send_amr_state_changed_func_t send_amr_state_changed_cb_;
  navigate_to_charging_func_t navigate_to_charging_cb_;
  slam_command_func_t slam_command_cb_;
  publish_twist_func_t publish_twist_cb_;
  std::shared_ptr<std::thread> return_charging_station_thread_;

  rclcpp::Logger logger_{rclcpp::get_logger("amr_state_machine")};

  void enter_idle_state();
  void enter_ready_state();
  void enter_on_ae_state();
  void enter_on_me_state();
  void enter_me_done_state();
  void enter_on_p2p_state(const Message &msg);
  void enter_on_follow_path_state(const Message &msg);
  void enter_on_waypoint_follow_path_state(const Message &msg);
  void enter_on_error_state();
  void enter_p2p_wait_state();
  void enter_follow_path_wait_state();
  void enter_low_power_charging_state();
  void enter_on_return_charging_state();
  void create_return_charging_station_thread();
  void return_charging_station_function(void *arg);
  void destory_return_charging_station_thread();

  void *get_current_path();
  void return_charging_station();

  bool handle_message(const Message &msg);
  void handle_msg();
  void handle_failed(const Message &msg);

  void save_map();
  void enter_localization_state();
  bool enter_localization_mode();
  bool get_relocalization_state();
  void start_rotation();
  void stop_rotation();
  void switch_charger_mode();
  void switch_navigation_mode();
  void notify_state_machine_changed();
  void send_me_finish_message();
  void send_me_message();
  void send_me_completed_message();
  void send_relocalization_pass_message();
  void send_return_charging_message();
  void update_state(int last_state, int state);

public:
  const static int IN_ACTIVE = 1;
  const static int IDLE = 2;
  const static int READY = 3;
  const static int ON_AE = 4;
  const static int ON_FOLLOW_PATH = 5;
  const static int FOLLOW_PATH_WAIT = 6;
  const static int ON_P2PNAV = 7;
  const static int P2PNAV_WAIT = 8;
  const static int ON_RETURN_CHARGING = 9;
  const static int LOW_POWER_CHARGING = 10;
  const static int ON_ERROR = 11;
  const static int ON_ME = 12;
  const static int ME_DONE = 13;
  const static int LOCALIZATION = 14;

  std::string get_current_state();
  void register_start_p2p_nav_callback(start_p2p_func_t cb);
  void register_start_follow_path_callback(start_follow_path_func_t cb);
  void register_start_waypoint_follow_path_callback(
      start_waypoint_follow_path_func_t cb);
  void register_sub_cmd_callback(sub_cmd_func_t cb);
  void register_start_charging_callback(start_charging_func_t cb);
  void register_notify_exception_callback(notify_exception_func_t cb);
  void
  register_send_amr_state_changed_callback(send_amr_state_changed_func_t cb);
  void register_navigate_to_charging_callback(navigate_to_charging_func_t cb);
  void register_slam_command_callback(slam_command_func_t cb);
  void register_publish_twist_callback(publish_twist_func_t cb);

  void process_cmd(int cmd, void *buffer = nullptr, size_t len = -1);
  void process_cmd(int cmd, uint32_t goal_id, vector<uint32_t> &ids);
  void process_sub_cmd(int message);
  void process_event(int event);
  void process_event(int event, uint8_t error_code);

  void init_amr();
  void release_amr();
  bool load_map();
  bool start_mapping();
  bool stop_mapping();
  bool check_potential_state(int cmd);

  AMRStateMachine();
  ~AMRStateMachine();

  std::string state_to_string(int msg) {
    std::string message;
    switch (msg) {
    case IN_ACTIVE:
      message = "IN_ACTIVE";
      break;
    case IDLE:
      message = "IDLE";
      break;
    case READY:
      message = "READY";
      break;
    case ON_AE:
      message = "ON_AE";
      break;
    case ON_ME:
      message = "ON_ME";
      break;
    case ME_DONE:
      message = "ME_DONE";
      break;
    case ON_FOLLOW_PATH:
      message = "ON_FOLLOW_PATH";
      break;
    case FOLLOW_PATH_WAIT:
      message = "FOLLOW_PATH_WAIT";
      break;
    case ON_P2PNAV:
      message = "ON_P2PNAV";
      break;
    case P2PNAV_WAIT:
      message = "P2PNAV_WAIT";
      break;
    case ON_RETURN_CHARGING:
      message = "ON_RETURN_CHARGING";
      break;
    case LOW_POWER_CHARGING:
      message = "LOW_POWER_CHARGING";
      break;
    case ON_ERROR:
      message = "ON_ERROR";
      break;
    case LOCALIZATION:
      message = "LOCALIZATION";
      break;
    default:
      message = "INVALID_STATE";
      break;
    }
    return message;
  }
};
} // namespace amr_manager
} // namespace qrb
#endif // AMR_STATE_MACHINE_HPP_