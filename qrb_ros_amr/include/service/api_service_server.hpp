/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_AMR__API_SERVICE_SERVER_HPP_
#define QRB_ROS_AMR__API_SERVICE_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "qrb_ros_amr_msgs/srv/api.hpp"
#include "amr_controller.hpp"
#include "developer_mode_subscriber.hpp"
#include "amr_manager.hpp"
#include <memory>
#include <string>

using namespace qrb::amr_manager;

namespace qrb_ros
{
namespace amr
{

class APIServiceServer : public rclcpp::Node
{
private:
  rclcpp::Service<qrb_ros_amr_msgs::srv::API>::SharedPtr srv_ptr_;
  std::shared_ptr<AMRManager> amr_manager_;
  std::shared_ptr<DeveloperModeSubscriber> dev_sub_;

  void init_service();
  void handle_api(const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<qrb_ros_amr_msgs::srv::API::Request> request,
      std::shared_ptr<qrb_ros_amr_msgs::srv::API::Response> response);

public:
  APIServiceServer(std::shared_ptr<AMRManager> & amr_manager,
      std::shared_ptr<DeveloperModeSubscriber> & dev_sub,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~APIServiceServer();
};
}  // namespace amr
}  // namespace qrb_ros
#endif  // QRB_ROS_AMR__API_SERVICE_SERVER_HPP_