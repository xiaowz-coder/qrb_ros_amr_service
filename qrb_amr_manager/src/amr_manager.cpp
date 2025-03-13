/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <pthread.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <utility>

#include "amr_manager.hpp"

namespace qrb {
namespace amr_manager {

AMRManager::AMRManager() {
  printf("%s, AMRManager", logger_);
  init();
}

AMRManager::~AMRManager() {}

void AMRManager::register_start_p2p_nav_callback(start_p2p_func_t cb) {
  state_machine_->register_start_p2p_nav_callback(cb);
}

void AMRManager::register_start_follow_path_callback(
    start_follow_path_func_t cb) {
  state_machine_->register_start_follow_path_callback(cb);
}

void AMRManager::register_start_waypoint_follow_path_callback(
    start_waypoint_follow_path_func_t cb) {
  state_machine_->register_start_waypoint_follow_path_callback(cb);
}

void AMRManager::register_change_mode_callback(change_mode_func_t cb) {
  low_power_->register_change_mode_callback(cb);
}

void AMRManager::register_sub_cmd_callback(sub_cmd_func_t cb) {
  state_machine_->register_sub_cmd_callback(cb);
}

void AMRManager::register_start_charging_callback(start_charging_func_t cb) {
  state_machine_->register_start_charging_callback(cb);
}

void AMRManager::register_notify_exception_callback(
    notify_exception_func_t cb) {
  state_machine_->register_notify_exception_callback(cb);
}

void AMRManager::register_send_amr_state_changed_callback(
    send_amr_state_changed_func_t cb) {
  state_machine_->register_send_amr_state_changed_callback(cb);
}

void AMRManager::register_navigate_to_charging_callback(
    navigate_to_charging_func_t cb) {
  state_machine_->register_navigate_to_charging_callback(cb);
}

void AMRManager::register_slam_command_callback(slam_command_func_t cb) {
  state_machine_->register_slam_command_callback(cb);
}

void AMRManager::register_publish_twist_callback(publish_twist_func_t cb) {
  state_machine_->register_publish_twist_callback(cb);
}

void AMRManager::process_event(int event) {
  state_machine_->process_event(event, 0);
}

void AMRManager::process_event(int event, int error_code) {
  state_machine_->process_event(event, (uint8_t)error_code);
}

bool AMRManager::check_potential_state(int cmd) {
  return state_machine_->check_potential_state(cmd);
}

void AMRManager::process_cmd(int cmd, void *buffer, size_t len) {
  state_machine_->process_cmd(cmd, buffer, len);
}

void AMRManager::process_cmd(int cmd, uint32_t goal_id, vector<uint32_t> &ids) {
  state_machine_->process_cmd(cmd, goal_id, ids);
}

void AMRManager::init_amr() { state_machine_->init_amr(); }

void AMRManager::release_amr() { state_machine_->release_amr(); }

void AMRManager::me_completed() {
  state_machine_->process_event(Message::ME_COMPLETED, 0);
}

void AMRManager::process_sub_cmd(int msg) {
  state_machine_->process_sub_cmd(msg);
}

void AMRManager::notify_amr_state_changed(int state) {
  low_power_->notify_amr_state_changed(state);
}

void AMRManager::notify_battery_changed(float battery_vol) {
  low_power_->notify_battery_changed(battery_vol);
}

void AMRManager::notify_charging_state_changed(uint8_t state) {
  low_power_->notify_charging_state_changed(state);
}

bool AMRManager::load_map() { return state_machine_->load_map(); }

bool AMRManager::start_mapping() { return state_machine_->start_mapping(); }

bool AMRManager::stop_mapping() { return state_machine_->stop_mapping(); }

void AMRManager::init() {
  printf("%s, init", logger_);

  state_machine_ = std::shared_ptr<AMRStateMachine>(new AMRStateMachine());

  low_power_ =
      std::shared_ptr<LowPowerManager>(new LowPowerManager(state_machine_));
}
} // namespace amr_manager
} // namespace qrb
