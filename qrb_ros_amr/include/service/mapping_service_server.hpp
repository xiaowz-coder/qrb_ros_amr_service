/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_AMR__MAPPING_SERVICE_SERVER_HPP_
#define QRB_ROS_AMR__MAPPING_SERVICE_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "qrb_ros_amr_msgs/srv/mapping.hpp"
#include "amr_manager.hpp"
#include <memory>
#include <string>

using namespace qrb::amr_manager;
using Mapping = qrb_ros_amr_msgs::srv::Mapping;

namespace qrb_ros
{
namespace amr
{
enum class Mapping_Cmd
{
  LOAD_MAP = 1,
  START_MAPPING = 2,
  STOP_MAPPING = 3,
};

class MappingServiceServer : public rclcpp::Node
{
private:
  rclcpp::Service<qrb_ros_amr_msgs::srv::Mapping>::SharedPtr srv_ptr_;
  std::shared_ptr<AMRManager> amr_manager_;

  void init_server();
  void handle_mapping(const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<qrb_ros_amr_msgs::srv::Mapping::Request> request,
      std::shared_ptr<qrb_ros_amr_msgs::srv::Mapping::Response> response);

  rclcpp::Logger logger_{ rclcpp::get_logger("mapping_service_server") };

public:
  MappingServiceServer(std::shared_ptr<AMRManager> & amr_manager,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MappingServiceServer();
};
}  // namespace amr
}  // namespace qrb_ros
#endif  // QRB_ROS_AMR__MAPPING_SERVICE_SERVER_HPP_