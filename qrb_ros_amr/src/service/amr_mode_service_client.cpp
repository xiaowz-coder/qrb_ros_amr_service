/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "amr_mode_service_client.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>

using namespace std::chrono_literals;

namespace qrb_ros
{
namespace amr
{
AMRModeServiceClient::AMRModeServiceClient(
  std::shared_ptr<AMRManager>& amr_manager, const rclcpp::NodeOptions& options)
  : Node("amr_mode", options),
  amr_manager_(amr_manager)
{
  client_ = this->create_client<SetBaseControlMode>("set_control_mode");
  execute_result_ = false;

  change_mode_callback_ = [&](uint8_t mode) {
    change_mode(mode);
  };
  amr_manager_->register_change_mode_callback(change_mode_callback_);
}

AMRModeServiceClient::~AMRModeServiceClient()
{
}

void AMRModeServiceClient::send_request(
    const SetBaseControlMode::Request::SharedPtr request)
{
  while (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(logger_, "service not available, waiting again...");
  }
  using ServiceResponseFuture =
      rclcpp::Client<SetBaseControlMode>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
    std::unique_lock<std::mutex> lck(mtx_);
    auto result = future.get();
    execute_result_ = result->result;
    cv_.notify_one();
  };
  auto future_result = client_->async_send_request(request, response_received_callback);
  std::unique_lock<std::mutex> lck(mtx_);
  cv_.wait(lck);
}

bool AMRModeServiceClient::change_mode(uint8_t mode)
{
  RCLCPP_INFO(logger_, "change mode: %d", mode);
  auto request = std::make_shared<SetBaseControlMode::Request>();
  request->mode = mode;
  send_request(request);
  return execute_result_;
}

bool AMRModeServiceClient::reset_odometer()
{
  RCLCPP_INFO(logger_, "reset odom");
  //TODO
  return execute_result_;
}
}  // namespace amr
}  // namespace qrb_ros
