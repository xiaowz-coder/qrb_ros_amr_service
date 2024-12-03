/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef AMR_MANAGER_HPP_
#define AMR_MANAGER_HPP_

#include <mutex>
#include <string.h>

#include "amr_state_machine.hpp"
#include "low_power_manager.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qrb {
namespace amr_manager {

typedef std::function<void(void* buffer)> start_p2p_func_t;
typedef std::function<void(void* path)> start_follow_path_func_t;
typedef std::function<void(uint8_t goal, std::vector<uint32_t>& ids)> start_waypoint_follow_path_func_t;
typedef std::function<void(bool p2p, uint8_t sub_cmd)> sub_cmd_func_t;
typedef std::function<void(bool start)> start_charging_func_t;
typedef std::function<void(uint8_t mode)> change_mode_func_t;
typedef std::function<void(bool exception, int error_code)> notify_exception_func_t;
typedef std::function<void(int state)> send_amr_state_changed_func_t;
typedef std::function<void(void)> navigate_to_charging_func_t;
typedef std::function<void(uint8_t cmd, bool& result)> slam_command_func_t;
typedef std::function<void(geometry_msgs::msg::Twist& velocity)> publish_twist_func_t;

/**
 * @class navigation_controller::AMRManager
 * @desc The AMRManager create nodes to control the Navigation.
 */
class AMRManager {
public:
  /**
   * @desc A constructor for AMRManager
   */
  AMRManager();

  /**
   * @desc A destructor for AMRManager
   */
  ~AMRManager();

  // ExceptionSubscriber
  void process_event(int event);
  void process_event(int event, int error_code);

  // CmdActionServer
  bool check_potential_state(int cmd);
  void process_cmd(int cmd, void *buffer, size_t len);
  void process_cmd(int cmd, uint32_t goal_id, std::vector<uint32_t>& ids);

  // APIServiceServer
  void init_amr();
  void release_amr();
  void me_completed();

  // SubCmdServer
  void process_sub_cmd(int msg);

  // AMRStatusTransporter
  void notify_amr_state_changed(int state);
  void notify_battery_changed(float battery_vol);
  void notify_charging_state_changed(uint8_t state);
  void register_start_charging_callback(start_charging_func_t cb);
  void register_notify_exception_callback(notify_exception_func_t cb);
  void
  register_send_amr_state_changed_callback(send_amr_state_changed_func_t cb);
  void register_publish_twist_callback(publish_twist_func_t cb);

  // NavigationActionClient
  void register_start_p2p_nav_callback(start_p2p_func_t cb);
  void register_start_follow_path_callback(start_follow_path_func_t cb);
  void register_start_waypoint_follow_path_callback(
      start_waypoint_follow_path_func_t cb);
  void register_sub_cmd_callback(sub_cmd_func_t cb);
  void register_navigate_to_charging_callback(navigate_to_charging_func_t cb);

  // AMRModeServiceClient
  void register_change_mode_callback(change_mode_func_t cb);

  // MappingServiceServer
  bool start_mapping();
  bool stop_mapping();
  bool load_map();

  // CartographerServiceClient
  void register_slam_command_callback(slam_command_func_t cb);

private:
  std::shared_ptr<AMRStateMachine> state_machine_;
  std::shared_ptr<LowPowerManager> low_power_;

  void init();

  rclcpp::Logger logger_{rclcpp::get_logger("amr_manager")};
};
} // namespace amr_manager
} // namespace qrb
#endif // AMR_MANAGER_HPP_