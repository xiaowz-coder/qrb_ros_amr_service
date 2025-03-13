/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cartographer_service_client.hpp"

constexpr char const * node_name = "cartographer_service_client";
constexpr char const * service_name = "qrb_slam_command";
constexpr char const * virtual_path_service_name = "virtual_path";

namespace qrb_ros
{
namespace amr
{
CartographerServiceClient::CartographerServiceClient(std::shared_ptr<AMRManager> & amr_manager)
  : Node(node_name), amr_manager_(amr_manager)
{
  RCLCPP_INFO(logger_, "Creating");
  init_client();

  slam_command_callback_ = [&](uint8_t cmd, bool & result) { result = handle_slam_command(cmd); };
  amr_manager_->register_slam_command_callback(slam_command_callback_);
}

bool CartographerServiceClient::handle_slam_command(uint8_t cmd)
{
  if (cmd == (uint8_t)Slam_Command::StartMapping) {
    return start_mapping();
  } else if (cmd == (uint8_t)Slam_Command::StopMapping) {
    return stop_mapping();
  } else if (cmd == (uint8_t)Slam_Command::SaveMap) {
    return save_map();
  } else if (cmd == (uint8_t)Slam_Command::LoadMap) {
    return load_map();
  } else if (cmd == (uint8_t)Slam_Command::StartLocalization) {
    return start_localization();
  } else if (cmd == (uint8_t)Slam_Command::Relocalization) {
    return relocalization();
  }

  RCLCPP_ERROR(logger_, "cmd(%d) is error", cmd);
  return false;
}

CartographerServiceClient::~CartographerServiceClient()
{
  RCLCPP_INFO(logger_, "Destroying");
}

bool CartographerServiceClient::start_mapping()
{
  RCLCPP_INFO(logger_, "start_mapping");
  if (!remove_waypoint_and_virtual_path()) {
    RCLCPP_ERROR(logger_, "Remove waypoint and virtual path failed");
    return false;
  }

  auto request = std::make_shared<SlamCommand::Request>();
  request->command_id = CommandCode::START_MAPPING;

  return send_api_request(request);
}

bool CartographerServiceClient::stop_mapping()
{
  RCLCPP_INFO(logger_, "stop_mapping");
  auto request = std::make_shared<SlamCommand::Request>();
  request->command_id = CommandCode::FINISH_MAPPING;

  return send_api_request(request);
}

bool CartographerServiceClient::save_map()
{
  RCLCPP_INFO(logger_, "save_map");
  auto request = std::make_shared<SlamCommand::Request>();
  request->command_id = CommandCode::SAVE_MAP;

  return send_api_request(request);
}

bool CartographerServiceClient::load_map()
{
  RCLCPP_INFO(logger_, "load_map");
  auto request = std::make_shared<SlamCommand::Request>();
  request->command_id = CommandCode::LOAD_MAP;

  return send_api_request(request);
}

bool CartographerServiceClient::start_localization()
{
  RCLCPP_INFO(logger_, "start_localization");
  auto request = std::make_shared<SlamCommand::Request>();
  request->command_id = CommandCode::START_LOCALIZATION;

  return send_api_request(request);
}

bool CartographerServiceClient::relocalization()
{
  RCLCPP_INFO(logger_, "relocalization");
  auto request = std::make_shared<SlamCommand::Request>();
  request->command_id = CommandCode::VALIDATE_RELOCALIZATION;

  return send_api_request(request);
}

bool CartographerServiceClient::send_api_request(SlamCommand::Request::SharedPtr request)
{
  if (!wait_for_service()) {
    return false;
  }

  return send_and_wait_for_request(request);
}

bool CartographerServiceClient::wait_for_service()
{
  uint32_t wait = 0;
  // wait for connectting service
  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(logger_, "service not available, waiting again...");
    wait++;
    if (wait >= SERVICE_TIMEOUT_DURATION) {
      RCLCPP_ERROR(logger_, "Wait for service timeout, Exiting.");
      return false;
    }
  }
  return true;
}

bool CartographerServiceClient::send_and_wait_for_request(SlamCommand::Request::SharedPtr request)
{
  using ServiceResponseFuture = rclcpp::Client<SlamCommand>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
    std::unique_lock<std::mutex> lck(mtx_);
    auto result = future.get();
    response_result_ = result->response.result;
    cv_.notify_one();
  };
  auto future_result = client_->async_send_request(request, response_received_callback);
  std::unique_lock<std::mutex> lck(mtx_);
  cv_.wait(lck);

  if (response_result_) {
    RCLCPP_INFO(logger_, "api is excuted successful");
  } else {
    RCLCPP_ERROR(logger_, "api is excuted failed");
  }
  return response_result_;
}

void CartographerServiceClient::init_client()
{
  RCLCPP_INFO(logger_, "init_client");
  client_ = this->create_client<SlamCommand>(service_name);
  virtual_path_client_ = this->create_client<VirtualPath>(virtual_path_service_name);
}

bool CartographerServiceClient::remove_waypoint_and_virtual_path()
{
  RCLCPP_INFO(logger_, "remove_waypoint_and_virtual_path");
  auto request = std::make_shared<VirtualPath::Request>();
  request->api_id = RemoveWaypointAndVirtualPath;

  return send_api_request2(request);
}

bool CartographerServiceClient::send_api_request2(VirtualPath::Request::SharedPtr request)
{
  if (!wait_for_service2()) {
    return false;
  }

  return send_and_wait_for_response2(request);
}

bool CartographerServiceClient::wait_for_service2()
{
  uint32_t wait = 0;
  // wait for connectting service
  while (!virtual_path_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(logger_, "service not available, waiting again...");

    wait++;
    if (wait >= SERVICE_TIMEOUT_DURATION) {
      RCLCPP_ERROR(logger_, "Wait for service timeout, Exiting.");
      return false;
    }
  }
  return true;
}

bool CartographerServiceClient::send_and_wait_for_response2(VirtualPath::Request::SharedPtr request)
{
  using ServiceResponseFuture = rclcpp::Client<VirtualPath>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
    std::unique_lock<std::mutex> lck(mtx_);
    auto result = future.get();
    virtual_path_response_result_ = result->result;
    cv_.notify_one();
  };

  auto future_result =
      virtual_path_client_->async_send_request(request, response_received_callback);
  std::unique_lock<std::mutex> lck(mtx_);
  cv_.wait(lck);

  if (virtual_path_response_result_ > 0) {
    RCLCPP_INFO(logger_, "handle virutla path successful");
    return true;
  }
  RCLCPP_ERROR(logger_, "handle virutla path failed");
  return false;
}
}  // namespace amr
}  // namespace qrb_ros