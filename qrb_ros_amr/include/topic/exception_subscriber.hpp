/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef EXCEPTION_SUBSCRIBER_HPP_
#define EXCEPTION_SUBSCRIBER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "amr_manager.hpp"
#include "qrb_ros_robot_base_msgs/msg/error.hpp"
#include <memory>

using namespace qrb::amr_manager;

namespace qrb_ros
{
namespace amr
{
class ExceptionSubscriber : public rclcpp::Node
{
private:
  rclcpp::Subscription<qrb_ros_robot_base_msgs::msg::Error>::SharedPtr sub_;
  std::shared_ptr<AMRManager> amr_manager_;
  rclcpp::Logger logger_{ rclcpp::get_logger("exception_subscriber") };

  void subscriber_callback(const qrb_ros_robot_base_msgs::msg::Error::SharedPtr msg);
  void init_subscription();

public:
  ExceptionSubscriber(
      std::shared_ptr<AMRManager>& amr_manager,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ExceptionSubscriber();

  void handle_debug_msg(uint8_t error);

};
}  // namespace amr
}  // namespace qrb_ros
#endif // EXCEPTION_SUBSCRIBER_HPP_