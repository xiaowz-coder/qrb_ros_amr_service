/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "rclcpp/rclcpp.hpp"
#include "exception_subscriber.hpp"
#include "amr_manager.hpp"

namespace qrb_ros
{
namespace amr
{
ExceptionSubscriber::ExceptionSubscriber(std::shared_ptr<AMRManager> & amr_manager,
    const rclcpp::NodeOptions & options)
  : Node("exception_sub", options), amr_manager_(amr_manager)
{
  init_subscription();
}

ExceptionSubscriber::~ExceptionSubscriber() {}

void ExceptionSubscriber::init_subscription()
{
  using namespace std::placeholders;
  sub_ = create_subscription<qrb_ros_robot_base_msgs::msg::Error>(
      "robot_base_error", 10, std::bind(&ExceptionSubscriber::subscriber_callback, this, _1));
}

void ExceptionSubscriber::subscriber_callback(
    const qrb_ros_robot_base_msgs::msg::Error::SharedPtr msg)
{
  int message;
  uint8_t error = msg->type;

  if (error == qrb_ros_robot_base_msgs::msg::Error::ERROR_WATCHDOG) {
    message = Message::AMR_EXCEPTION;
    RCLCPP_ERROR(logger_, "Receive watchdog error");
  } else if (error == qrb_ros_robot_base_msgs::msg::Error::ERROR_MOTOR) {
    message = Message::AMR_EXCEPTION;
    RCLCPP_ERROR(logger_, "Receive motor error");
  } else if (error == qrb_ros_robot_base_msgs::msg::Error::ERROR_OTHER) {
    RCLCPP_ERROR(logger_, "Receive other error");
    return;
  } else if (error == qrb_ros_robot_base_msgs::msg::Error::ERROR_CHARGER) {
    RCLCPP_ERROR(logger_, "Receive changer error");
    return;
  } else {
    RCLCPP_ERROR(logger_, "Receive message = %d", error);
    message = Message::AMR_NORMAL;
  }
  amr_manager_->process_event(message, error);
}

void ExceptionSubscriber::handle_debug_msg(uint8_t error)
{
  int message;

  if (error == qrb_ros_robot_base_msgs::msg::Error::ERROR_WATCHDOG) {
    message = Message::AMR_EXCEPTION;
    RCLCPP_ERROR(logger_, "Receive watchdog error");
  } else if (error == qrb_ros_robot_base_msgs::msg::Error::ERROR_MOTOR) {
    message = Message::AMR_EXCEPTION;
    RCLCPP_ERROR(logger_, "Receive motor error");
  } else if (error == qrb_ros_robot_base_msgs::msg::Error::ERROR_OTHER) {
    RCLCPP_ERROR(logger_, "Receive other error");
    return;
  } else if (error == qrb_ros_robot_base_msgs::msg::Error::ERROR_CHARGER) {
    RCLCPP_ERROR(logger_, "Receive changer error");
    return;
  } else {
    RCLCPP_ERROR(logger_, "Receive message = %d", error);
    message = Message::AMR_NORMAL;
  }
  RCLCPP_INFO(logger_, "error = %d", error);
  amr_manager_->process_event(message, error);
}
}  // namespace amr
}  // namespace qrb_ros
