/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_AMR_MANAGER__AMR_STATE_MACHINE_HPP_
#define QRB_AMR_MANAGER__AMR_STATE_MACHINE_HPP_

#include "message_queue.hpp"

#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "common.hpp"

using namespace std;

#define ANGULAR_VELOCITY 0.19  // the min angular velocity for arm

namespace qrb {
namespace amr_manager {

class AMRStateMachine {
private:
  std::mutex mtx_;
  std::condition_variable * cv_;
  bool * sub_cmd_success_;
  void * path_buffer_;
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

  const char* logger_ = "amr_state_machine";

  void enter_idle_state();
  void enter_ready_state();
  void enter_on_ae_state();
  void enter_on_me_state();
  void enter_me_done_state();
  void enter_on_p2p_state(const Message & msg);
  void enter_on_follow_path_state(const Message & msg);
  void enter_on_waypoint_follow_path_state(const Message & msg);
  void enter_on_error_state();
  void enter_p2p_wait_state();
  void enter_follow_path_wait_state();
  void enter_low_power_charging_state();
  void enter_on_return_charging_state();

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
#endif // QRB_AMR_MANAGER__AMR_STATE_MACHINE_HPP_