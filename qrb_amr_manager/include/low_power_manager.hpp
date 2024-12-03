/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef LOW_POWER_MANAGER_HPP_
#define LOW_POWER_MANAGER_HPP_

#include "amr_state_machine.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

namespace qrb {
namespace amr_manager {

enum class ChargeCmd {
  START = 0, // start charging
  END = 1,   // end charging
};

enum class ChargerState {
  UNKNOWN = 0,
  IDLE = 3,           // charging module is idle state
  SEARCHING = 5,      // charging module is searching the infrared
  CONTROLLING = 6,    // charging module is controlling the robot
  FORCE_CHARGING = 7, // charging can not be interrupted
  CHARGING = 1,
  CHARGING_DONE = 4, // completed charging
  ERROR = 8,
};

enum class SetControlMode {
  APPLICATION = 0,
  CHARGER = 1,
  REMOTE_CONTROLLER = 2,
};

class LowPowerManager {
private:
  std::shared_ptr<AMRStateMachine> state_machine_;
  void send_event(const int event);
  void set_charger_mode();
  void set_navigation_mode();
  int get_amr_state();

  float current_battery_voltage_level_;
  uint8_t charging_station_;
  std::mutex mtx_;
  int amr_state_;
  change_mode_func_t change_mode_cb_;
  rclcpp::Logger logger_{rclcpp::get_logger("low_power_manager")};

public:
  void register_change_mode_callback(change_mode_func_t cb);
  void notify_amr_state_changed(int state);
  void notify_battery_changed(float battery_vol);
  void notify_charging_state_changed(uint8_t state);
  LowPowerManager(std::shared_ptr<AMRStateMachine> &state_machine);
  ~LowPowerManager();
};
} // namespace amr_manager
} // namespace qrb
#endif // LOW_POWER_MANAGER_HPP_