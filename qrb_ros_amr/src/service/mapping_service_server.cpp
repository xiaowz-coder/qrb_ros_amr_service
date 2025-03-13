/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "mapping_service_server.hpp"

namespace qrb_ros
{
namespace amr
{
MappingServiceServer::MappingServiceServer(std::shared_ptr<AMRManager> & amr_manager,
    const rclcpp::NodeOptions & options)
  : Node("mapping_server", options), amr_manager_(amr_manager)
{
  RCLCPP_INFO(logger_, "MappingServiceServer");
  init_server();
}

MappingServiceServer::~MappingServiceServer() {}

void MappingServiceServer::init_server()
{
  using namespace std::placeholders;
  srv_ptr_ = this->create_service<qrb_ros_amr_msgs::srv::Mapping>(
      "amr_mapping", std::bind(&MappingServiceServer::handle_mapping, this, _1, _2, _3));
}

void MappingServiceServer::handle_mapping(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<qrb_ros_amr_msgs::srv::Mapping::Request> request,
    std::shared_ptr<qrb_ros_amr_msgs::srv::Mapping::Response> response)
{
  uint8_t cmd = request->cmd;
  RCLCPP_INFO(logger_, "handle_mapping, cmd=%d", cmd);

  bool result;
  if (cmd == (uint8_t)Mapping_Cmd::START_MAPPING) {
    result = amr_manager_->start_mapping();
  } else if (cmd == (uint8_t)Mapping_Cmd::STOP_MAPPING) {
    result = amr_manager_->stop_mapping();
  } else if (cmd == (uint8_t)Mapping_Cmd::LOAD_MAP) {
    result = amr_manager_->load_map();
  }
  response->result = result;
}
}  // namespace amr
}  // namespace qrb_ros