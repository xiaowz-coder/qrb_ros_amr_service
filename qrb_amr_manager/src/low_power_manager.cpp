/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "low_power_manager.hpp"

#define LOW_POWER_VOLTAGE_LEVEL 22

namespace qrb
{
namespace amr_manager
{

LowPowerManager::LowPowerManager(std::shared_ptr<AMRStateMachine> & state_machine)
  : state_machine_(state_machine)
{
}

LowPowerManager::~LowPowerManager() {}

void LowPowerManager::register_change_mode_callback(change_mode_func_t cb)
{
  printf("%s, register_change_mode_callback", logger_);
  change_mode_cb_ = cb;
}

void LowPowerManager::notify_amr_state_changed(int state)
{
  printf("%s, notify_amr_state_changed", logger_);
  std::unique_lock<std::mutex> lck(mtx_);
  amr_state_ = state;
}

void LowPowerManager::notify_battery_changed(float battery_vol)
{
  if (battery_vol <= LOW_POWER_VOLTAGE_LEVEL) {
    printf("%s, Start return charging station", logger_);
    send_event(Message::LOW_POWER);
  }
  current_battery_voltage_level_ = battery_vol;
}

void LowPowerManager::notify_charging_state_changed(uint8_t state)
{
  charging_station_ = state;
  switch (state) {
    case (uint8_t)ChargerState::IDLE:
      printf("%s, Charger idle", logger_);
      set_navigation_mode();
      break;
    case (uint8_t)ChargerState::SEARCHING:
      printf("%s, Searching...", logger_);
      break;
    case (uint8_t)ChargerState::CONTROLLING:
      printf("%s, Controlling...", logger_);
      set_charger_mode();
      break;
    case (uint8_t)ChargerState::FORCE_CHARGING:
      printf("%s, Attached charger pie", logger_);
      send_event(Message::RETURN_CHARGING_FINISH);
      break;
    case (uint8_t)ChargerState::CHARGING:
      printf("%s, Charging...", logger_);
      send_event(Message::NORMAL_POWER);
      set_navigation_mode();
      break;
    case (uint8_t)ChargerState::CHARGING_DONE:
      printf("%s, Charging done", logger_);
      set_navigation_mode();
      break;
    case (uint8_t)ChargerState::ERROR:
      printf("%s, Charging station is error", logger_);
      set_navigation_mode();
      break;
    default:
      break;
  }
}

void LowPowerManager::send_event(const int event)
{
  state_machine_->process_event(event);
}

void LowPowerManager::set_charger_mode()
{
  if (get_amr_state() == AMRStateMachine::ON_RETURN_CHARGING) {
    change_mode_cb_((uint8_t)SetControlMode::CHARGER);
  }
}

void LowPowerManager::set_navigation_mode()
{
  change_mode_cb_((uint8_t)SetControlMode::APPLICATION);
}

int LowPowerManager::get_amr_state()
{
  std::unique_lock<std::mutex> lck(mtx_);
  return amr_state_;
}
}  // namespace amr_manager
}  // namespace qrb