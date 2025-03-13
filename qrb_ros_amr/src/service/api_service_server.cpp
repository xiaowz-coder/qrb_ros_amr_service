/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "rclcpp/rclcpp.hpp"
#include "qrb_ros_amr_msgs/srv/api.hpp"
#include "nav_msgs/msg/path.hpp"
#include "api_service_server.hpp"
#include "amr_controller.hpp"
#include "cmd_action_server.hpp"

namespace qrb_ros
{
namespace amr
{
APIServiceServer::APIServiceServer(std::shared_ptr<AMRManager> & amr_manager,
    std::shared_ptr<DeveloperModeSubscriber> & dev_sub,
    const rclcpp::NodeOptions & options)
  : Node("api_server", options), amr_manager_(amr_manager), dev_sub_(dev_sub)
{
  init_service();
}

APIServiceServer::~APIServiceServer() {}

void APIServiceServer::init_service()
{
  using namespace std::placeholders;
  srv_ptr_ = create_service<qrb_ros_amr_msgs::srv::API>(
      "api", std::bind(&APIServiceServer::handle_api, this, _1, _2, _3));
}

void APIServiceServer::handle_api(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<qrb_ros_amr_msgs::srv::API::Request> request,
    std::shared_ptr<qrb_ros_amr_msgs::srv::API::Response> response)
{
  bool succeed = amr_manager_->check_potential_state(Command::OTHER);
  if (!succeed) {
    RCLCPP_INFO(this->get_logger(), "current state is error");
    response->result = false;
    return;
  }

  int api = request->api_id;

  switch (api) {
    case API::INIT_AMR:
      amr_manager_->init_amr();
      break;
    case API::RELEASE_AMR:
      amr_manager_->release_amr();
      break;
    case API::ME_COMPLETED:
      amr_manager_->me_completed();
      break;
    case API::ENABLE_DEVELOPER_MODE: {
      bool val = request->developer_mode;
      dev_sub_->set_developer_mode(val);
      break;
    }
    default:
      RCLCPP_INFO(this->get_logger(), "Incoming request api_id unknown");
      break;
  }
  RCLCPP_INFO(
      this->get_logger(), "Incoming request api_id: %s", API::to_string(request->api_id).c_str());
  response->result = true;
  return;
}
}  // namespace amr
}  // namespace qrb_ros