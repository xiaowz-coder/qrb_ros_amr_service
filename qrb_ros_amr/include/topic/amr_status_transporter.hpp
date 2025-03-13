/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef AMR_STATUS_TRANSPORTER_HPP_
#define AMR_STATUS_TRANSPORTER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "qrb_ros_amr_msgs/msg/amr_status.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "qrb_ros_amr_msgs/msg/wheel_status.hpp"
#include "qrb_ros_amr_msgs/msg/battery_info.hpp"
#include "qrb_ros_robot_base_msgs/msg/charger_cmd.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "amr_manager.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "nav_2d_msgs/msg/twist2_d_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <memory>

using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using ChargerCmd = qrb_ros_robot_base_msgs::msg::ChargerCmd;

using namespace qrb::amr_manager;

namespace qrb_ros
{
namespace amr
{
class LowPowerManager;

/**
 * @enum amr_controller::StatusID
 * @desc Enum class representing the changed status.
 */
enum class StatusID
{
  ALL = 0,            // all items need be updated
  AMR_Exception = 1,  // AMR happens an exception or restores to normal
  Battery_Level = 2,  // Battery level of AMR is changed
  Pose = 3,           // Pose of AMR is changed
  State_Machine = 4,  // State of AMR is changed
  Velocity = 5,       // The linear or angular velocities of wheels is changed
};

class AMRStatusTransporter : public rclcpp::Node
{
private:
  rclcpp::Publisher<qrb_ros_amr_msgs::msg::AMRStatus>::SharedPtr pub_;
  rclcpp::Publisher<ChargerCmd>::SharedPtr charger_pub_;
  rclcpp::Subscription<qrb_ros_amr_msgs::msg::WheelStatus>::SharedPtr wheel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  std::shared_ptr<AMRManager> amr_manager_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string target_frame_ = "map";
  std::string source_frame_ = "base_link";
  PoseStamped source_pose_;
  PoseStamped target_pose_;
  PoseStamped last_pose_;
  rclcpp::TimerBase::SharedPtr timer_{ nullptr };
  std::mutex mtx_;
  bool active_ = false;
  bool tf_working_;
  uint64_t count_;

  qrb::amr_manager::start_charging_func_t start_charging_callback_;
  qrb::amr_manager::notify_exception_func_t notify_exception_callback_;
  qrb::amr_manager::send_amr_state_changed_func_t send_amr_state_changed_callback_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  publish_twist_func_t publish_twist_cb_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vel_sub_;
  nav_2d_msgs::msg::Twist2DStamped odom_velocity_;
  long last_time_;
  rclcpp::Logger logger_{ rclcpp::get_logger("amr_status_transporter") };

  void wheel_status_callback(const qrb_ros_amr_msgs::msg::WheelStatus::SharedPtr msg);
  void init_publisher();
  void init_subscription();
  void send_amr_status(const qrb_ros_amr_msgs::msg::AMRStatus msg);

  void init_tf_subscriber();
  void convert_tf_to_pose();
  void update_amr_pose(PoseStamped & pose);
  bool is_pose_change();
  bool is_equal(double a, double b);
  void pose_changed_callback(const PoseStamped::SharedPtr pose);
  void battery_status_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void send_velocity(twist_vel & velocity);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

public:
  void notify_exception_event(bool exception, uint8_t error_code);
  void send_amr_state_changed(int state);
  void update_map(OccupancyGrid map);
  void stop_charging();
  void start_charging();
  void notify_battery_changed(float voltage);
  AMRStatusTransporter(std::shared_ptr<AMRManager> & amr_manager,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AMRStatusTransporter();
  qrb_ros_amr_msgs::msg::AMRStatus message_;
};
}  // namespace amr
}  // namespace qrb_ros
#endif  // AMR_STATUS_TRANSPORTER_HPP_