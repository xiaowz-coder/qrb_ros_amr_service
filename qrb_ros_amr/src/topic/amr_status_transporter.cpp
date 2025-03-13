/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "amr_status_transporter.hpp"
#include "amr_manager.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "amr_manager/common.hpp"
#include <thread>
#include <pthread.h>

#include <stdio.h>
#include <sys/syscall.h>
#include <unistd.h>

using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using namespace std::chrono_literals;

namespace qrb_ros
{
namespace amr
{
AMRStatusTransporter::AMRStatusTransporter(std::shared_ptr<AMRManager> & amr_manager,
    const rclcpp::NodeOptions & options)
  : Node("amr_status_transporter", options), amr_manager_(amr_manager)
{
  RCLCPP_INFO(logger_, "AMRStatusTransporter");
  init_subscription();
  init_tf_subscriber();
  init_publisher();
  last_time_ = 0;
}

AMRStatusTransporter::~AMRStatusTransporter() {}

void AMRStatusTransporter::init_publisher()
{
  pub_ = create_publisher<qrb_ros_amr_msgs::msg::AMRStatus>("amr_status", 10);

  charger_pub_ = create_publisher<ChargerCmd>("charger_cmd", 10);

  start_charging_callback_ = [&](bool start) {
    if (start) {
      start_charging();
    } else {
      stop_charging();
    }
  };
  amr_manager_->register_start_charging_callback(start_charging_callback_);

  notify_exception_callback_ = [&](bool exception, uint8_t error_code) {
    if (exception) {
      notify_exception_event(true, error_code);
    } else {
      notify_exception_event(false, 0);
    }
  };
  amr_manager_->register_notify_exception_callback(notify_exception_callback_);

  send_amr_state_changed_callback_ = [&](int state) { send_amr_state_changed(state); };
  amr_manager_->register_send_amr_state_changed_callback(send_amr_state_changed_callback_);

  twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  publish_twist_cb_ = [&](twist_vel & velocity) { send_velocity(velocity); };
  amr_manager_->register_publish_twist_callback(publish_twist_cb_);
}

void AMRStatusTransporter::init_subscription()
{
  using namespace std::placeholders;
  wheel_sub_ = create_subscription<qrb_ros_amr_msgs::msg::WheelStatus>(
      "wheel_status", 10, std::bind(&AMRStatusTransporter::wheel_status_callback, this, _1));
  battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "battery", 10, std::bind(&AMRStatusTransporter::battery_status_callback, this, _1));
  pose_sub_ = create_subscription<PoseStamped>(
      "amr_pose", 10, std::bind(&AMRStatusTransporter::pose_changed_callback, this, _1));
  vel_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::SystemDefaultsQoS(),
      std::bind(&AMRStatusTransporter::odom_callback, this, std::placeholders::_1));
}

void AMRStatusTransporter::notify_exception_event(bool exception, uint8_t error_code)
{
  message_.header.stamp = get_clock()->now();
  message_.amr_exception = exception;
  message_.error_code = error_code;
  message_.status_change_id = (int)StatusID::AMR_Exception;
  send_amr_status(message_);
}

void AMRStatusTransporter::send_amr_status(const qrb_ros_amr_msgs::msg::AMRStatus msg)
{
  pub_->publish(msg);
}

void AMRStatusTransporter::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double vel_x = msg->twist.twist.linear.x;
  double vel_y = msg->twist.twist.linear.y;
  double vel_z = msg->twist.twist.angular.z;
  if (is_equal(vel_x, odom_velocity_.velocity.x) && is_equal(vel_y, odom_velocity_.velocity.y) &&
      is_equal(vel_z, odom_velocity_.velocity.theta)) {
    RCLCPP_DEBUG(logger_, "velocity is not changed");
    return;
  }

  long now = rclcpp::Clock().now().seconds();
  long duration = now - last_time_;
  if (duration < 1) {
    RCLCPP_DEBUG(logger_, "duration=%d", duration);
    return;
  }
  last_time_ = now;

  message_.status_change_id = (int)StatusID::Velocity;
  message_.vel.x = vel_x;
  message_.vel.y = vel_y;
  message_.vel.z = vel_z;
  send_amr_status(message_);
  odom_velocity_.velocity.x = vel_x;
  odom_velocity_.velocity.y = vel_y;
  odom_velocity_.velocity.theta = vel_z;
  RCLCPP_DEBUG(logger_, "velocity_x:%f,velocity_y:%f,velocity_theta:%f", vel_x, vel_y, vel_z);
}

void AMRStatusTransporter::wheel_status_callback(
    const qrb_ros_amr_msgs::msg::WheelStatus::SharedPtr msg)
{
  // TODO:
}

void AMRStatusTransporter::send_amr_state_changed(int state)
{
  RCLCPP_INFO(logger_, "send_amr_state_changed");
  message_.status_change_id = (int)StatusID::State_Machine;
  message_.current_state = state;
  send_amr_status(message_);
  amr_manager_->notify_amr_state_changed(state);
}

void AMRStatusTransporter::init_tf_subscriber()
{
  std::unique_lock<std::mutex> lck(mtx_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tf2::Quaternion q;
  q.setRPY(0.0f, 0.0f, 0.0f);  // yaw, pitch, roll

  // The center pose of the robot in the radar coordinate system.
  // if source_pose set header.stamp, the tf transform will error.
  source_pose_.pose.position.x = 0.0;
  source_pose_.pose.position.y = 0.0;
  source_pose_.pose.position.z = 0.0;
  source_pose_.pose.orientation.x = q.x();
  source_pose_.pose.orientation.y = q.y();
  source_pose_.pose.orientation.z = q.z();
  source_pose_.pose.orientation.w = q.w();
  source_pose_.header.frame_id = source_frame_;

  // the center pose of robot in the map corrdinate system.
  target_pose_.pose.position.x = 0.0;
  target_pose_.pose.position.y = 0.0;
  target_pose_.pose.position.z = 0.0;
  target_pose_.pose.orientation.x = q.w();
  target_pose_.pose.orientation.y = q.y();
  target_pose_.pose.orientation.z = q.z();
  target_pose_.pose.orientation.w = q.w();
  target_pose_.header.stamp = this->now();
  target_pose_.header.frame_id = target_frame_;

  timer_ = this->create_wall_timer(1s, std::bind(&AMRStatusTransporter::convert_tf_to_pose, this));
}

void AMRStatusTransporter::convert_tf_to_pose()
{
  std::unique_lock<std::mutex> lck(mtx_);
  try {
    // get the transforstamped that pose change from radar coordinate to map coordinate.
    geometry_msgs::msg::TransformStamped t =
        tf_buffer_->lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);

    // transform the robot center pose from radar coordinate to map coordinate.
    PoseStamped pose = tf_buffer_->transform(source_pose_, target_frame_, std::chrono::seconds(10));

    if (target_pose_.header.stamp == pose.header.stamp) {
      return;
    }

    target_pose_.pose.position.x = pose.pose.position.x;
    target_pose_.pose.position.y = pose.pose.position.y;
    target_pose_.pose.position.z = pose.pose.position.z;
    target_pose_.pose.orientation.x = pose.pose.orientation.x;
    target_pose_.pose.orientation.y = pose.pose.orientation.y;
    target_pose_.pose.orientation.z = pose.pose.orientation.z;
    target_pose_.pose.orientation.w = pose.pose.orientation.w;
    target_pose_.header.stamp = pose.header.stamp;
    target_pose_.header.frame_id = pose.header.frame_id;
    if (!is_pose_change()) {
      return;
    }

    RCLCPP_DEBUG(logger_, "transform pose(%f, %f, %f, %f, %f, %f, %f)",
        target_pose_.pose.position.x, target_pose_.pose.position.y, target_pose_.pose.position.z,
        target_pose_.pose.orientation.x, target_pose_.pose.orientation.y,
        target_pose_.pose.orientation.z, target_pose_.pose.orientation.w);

    RCLCPP_DEBUG(logger_, "transform pose: header:(%d, %s)", target_pose_.header.stamp,
        target_pose_.header.frame_id.c_str());

    tf_working_ = true;
    update_amr_pose(target_pose_);
    last_pose_ = target_pose_;
  } catch (const tf2::TransformException & ex) {
    if ((count_ % 60) == 0) {
      RCLCPP_ERROR(logger_, "Could not transform %s to %s: %s", source_frame_.c_str(),
          target_frame_.c_str(), ex.what());
    }
    tf_working_ = false;
    count_++;
    return;
  }
}

void AMRStatusTransporter::update_amr_pose(PoseStamped & pose)
{
  RCLCPP_DEBUG(logger_, "update_amr_pose");
  message_.status_change_id = (int)StatusID::Pose;
  message_.current_pose = pose;
  send_amr_status(message_);
}

bool AMRStatusTransporter::is_pose_change()
{
  if (is_equal(target_pose_.pose.position.x, last_pose_.pose.position.x) &&
      is_equal(target_pose_.pose.position.y, last_pose_.pose.position.y) &&
      is_equal(target_pose_.pose.position.z, last_pose_.pose.position.z) &&
      is_equal(target_pose_.pose.orientation.x, last_pose_.pose.orientation.x) &&
      is_equal(target_pose_.pose.orientation.y, last_pose_.pose.orientation.y) &&
      is_equal(target_pose_.pose.orientation.z, last_pose_.pose.orientation.z) &&
      is_equal(target_pose_.pose.orientation.w, last_pose_.pose.orientation.w)) {
    return false;
  }
  return true;
}

bool AMRStatusTransporter::is_equal(double a, double b)
{
  double delta = fabs(a - b);
  if (delta < 0.02) {
    return true;
  }
  return false;
}

void AMRStatusTransporter::pose_changed_callback(const PoseStamped::SharedPtr pose)
{
  std::unique_lock<std::mutex> lck(mtx_);
  target_pose_.pose.position.x = pose->pose.position.x;
  target_pose_.pose.position.y = pose->pose.position.y;
  target_pose_.pose.position.z = pose->pose.position.z;
  target_pose_.pose.orientation.x = pose->pose.orientation.x;
  target_pose_.pose.orientation.y = pose->pose.orientation.y;
  target_pose_.pose.orientation.z = pose->pose.orientation.z;
  target_pose_.pose.orientation.w = pose->pose.orientation.w;
  target_pose_.header.stamp = pose->header.stamp;
  target_pose_.header.frame_id = pose->header.frame_id;
  if (!tf_working_) {
    RCLCPP_INFO(logger_, "Update AMR pose(%f, %f, %f, %f, %f, %f, %f)",
        target_pose_.pose.position.x, target_pose_.pose.position.y, target_pose_.pose.position.z,
        target_pose_.pose.orientation.x, target_pose_.pose.orientation.y,
        target_pose_.pose.orientation.z, target_pose_.pose.orientation.w);
    update_amr_pose(target_pose_);
  }
}

void AMRStatusTransporter::battery_status_callback(
    const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  float voltage = msg->voltage;
  float current = msg->current;
  uint8_t state = msg->power_supply_status;
  message_.status_change_id = (int)StatusID::Battery_Level;
  message_.header = msg->header;
  message_.battery_vol = voltage;

  if (voltage != 0) {
    RCLCPP_INFO(
        logger_, "battery changed voltage=%.2f, current=%.2f, state=%d", voltage, current, state);
    amr_manager_->notify_battery_changed(voltage);
    amr_manager_->notify_charging_state_changed(state);
    send_amr_status(message_);
  }
}

void AMRStatusTransporter::notify_battery_changed(float voltage)
{
  message_.status_change_id = (int)StatusID::Battery_Level;
  message_.battery_vol = voltage;

  RCLCPP_INFO(logger_, "battery changed voltage=%.2f", voltage);
  if (voltage != 0) {
    amr_manager_->notify_battery_changed(voltage);
  }

  send_amr_status(message_);
}

void AMRStatusTransporter::start_charging()
{
  RCLCPP_INFO(logger_, "Start charging");
  ChargerCmd msg;
  msg.cmd == ChargerCmd::START_CHARGING;
  charger_pub_->publish(msg);
}

void AMRStatusTransporter::stop_charging()
{
  RCLCPP_INFO(logger_, "Stop charging");
  ChargerCmd msg;
  msg.cmd == ChargerCmd::STOP_CHARGING;
  charger_pub_->publish(msg);
}

void AMRStatusTransporter::send_velocity(twist_vel & velocity)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = velocity.x;
  twist.linear.y = velocity.y;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = velocity.z;
  RCLCPP_INFO(logger_, "send velocity(%.2f, %.2f, %.2f) to robot", twist.linear.x, twist.linear.y,
      twist.angular.z);
  twist_pub_->publish(twist);
}
}  // namespace amr
}  // namespace qrb_ros