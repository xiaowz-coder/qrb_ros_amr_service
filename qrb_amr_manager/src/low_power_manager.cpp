/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "low_power_manager.hpp"

#define LOW_POWER_VOLTAGE_LEVEL 22

namespace qrb {
namespace amr_manager {

LowPowerManager::LowPowerManager(
    std::shared_ptr<AMRStateMachine> &state_machine)
    : state_machine_(state_machine) {}

LowPowerManager::~LowPowerManager() {}

void LowPowerManager::register_change_mode_callback(change_mode_func_t cb) {
  RCLCPP_INFO(logger_, "register_change_mode_callback");
  change_mode_cb_ = cb;
}

void LowPowerManager::notify_amr_state_changed(int state) {
  RCLCPP_INFO(logger_, "notify_amr_state_changed");
  std::unique_lock<std::mutex> lck(mtx_);
  amr_state_ = state;
}

void LowPowerManager::notify_battery_changed(float battery_vol) {
  if (battery_vol <= LOW_POWER_VOLTAGE_LEVEL) {
    RCLCPP_INFO(logger_, "Start return charging station");
    send_event(Message::LOW_POWER);
  }
  current_battery_voltage_level_ = battery_vol;
}

void LowPowerManager::notify_charging_state_changed(uint8_t state) {
  charging_station_ = state;
  switch (state) {
    case (uint8_t)ChargerState::IDLE:
      RCLCPP_INFO(logger_, "Charger idle");
      set_navigation_mode();
      break;
    case (uint8_t)ChargerState::SEARCHING:
      RCLCPP_INFO(logger_, "Searching...");
      break;
    case (uint8_t)ChargerState::CONTROLLING:
      RCLCPP_INFO(logger_, "Controlling...");
      set_charger_mode();
      break;
    case (uint8_t)ChargerState::FORCE_CHARGING:
      RCLCPP_INFO(logger_, "Attached charger pie");
      send_event(Message::RETURN_CHARGING_FINISH);
      break;
    case (uint8_t)ChargerState::CHARGING:
      RCLCPP_INFO(logger_, "Charging...");
      send_event(Message::NORMAL_POWER);
      set_navigation_mode();
      break;
    case (uint8_t)ChargerState::CHARGING_DONE:
      RCLCPP_INFO(logger_, "Charging done");
      set_navigation_mode();
      break;
    case (uint8_t)ChargerState::ERROR:
      RCLCPP_ERROR(logger_, "Charging station is error");
      set_navigation_mode();
      break;
    default:
      break;
  }
}

void LowPowerManager::send_event(const int event) {
  state_machine_->process_event(event);
}

void LowPowerManager::set_charger_mode() {
  if (get_amr_state() == AMRStateMachine::ON_RETURN_CHARGING) {
    change_mode_cb_((uint8_t)SetControlMode::CHARGER);
  }
}

void LowPowerManager::set_navigation_mode() {
  change_mode_cb_((uint8_t)SetControlMode::APPLICATION);
}

int LowPowerManager::get_amr_state() {
  std::unique_lock<std::mutex> lck(mtx_);
  return amr_state_;
}
} // namespace amr_manager
} // namespace qrb