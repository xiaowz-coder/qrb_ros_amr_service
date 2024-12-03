/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "amr_controller.hpp"
#include <iostream>

namespace qrb_ros
{
namespace amr
{

AMRController::AMRController()
{
  init_nodes();
  RCLCPP_INFO(logger_, "Init amr node");
}

AMRController::~AMRController()
{
}

void AMRController::init_nodes()
{
  executor_ = std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>(
              new rclcpp::executors::MultiThreadedExecutor());
  amr_manager_ = std::shared_ptr<AMRManager>(
              new AMRManager());

  exception_sub_ = std::shared_ptr<ExceptionSubscriber>(
               new ExceptionSubscriber(amr_manager_));

  mapping_server_ = std::shared_ptr<MappingServiceServer>(
              new MappingServiceServer(amr_manager_));

  cmd_server_ = std::shared_ptr<CmdActionServer>(
               new CmdActionServer(amr_manager_));

  sub_cmd_server_ = std::shared_ptr<SubCmdServer>(
               new SubCmdServer(amr_manager_));

  auto_mapper_client_ = std::shared_ptr<AutoMapperActionClient>(
               new AutoMapperActionClient(amr_manager_));

  nav_client_ = std::shared_ptr<NavigationActionClient>(
               new NavigationActionClient(amr_manager_));

  cartographer_service_client_ = std::shared_ptr<CartographerServiceClient>(
               new CartographerServiceClient(amr_manager_));

  amr_mode_client_ = std::shared_ptr<AMRModeServiceClient>(
               new AMRModeServiceClient(amr_manager_));

  amr_status_transporter_ = std::shared_ptr<AMRStatusTransporter>(
               new AMRStatusTransporter(amr_manager_));

  dev_sub_ = std::shared_ptr<DeveloperModeSubscriber>(
              new DeveloperModeSubscriber(amr_manager_, exception_sub_, amr_status_transporter_));

  api_server_ = std::shared_ptr<APIServiceServer>(
              new APIServiceServer(amr_manager_, dev_sub_));

  executor_->add_node(api_server_);
  executor_->add_node(mapping_server_);
  executor_->add_node(dev_sub_);
  executor_->add_node(exception_sub_);
  executor_->add_node(cmd_server_);
  executor_->add_node(sub_cmd_server_);
  executor_->add_node(auto_mapper_client_);
  executor_->add_node(nav_client_);
  executor_->add_node(cartographer_service_client_);
  executor_->add_node(amr_mode_client_);
  executor_->add_node(amr_status_transporter_);
}
}  // namespace amr
}  // namespace qrb_ros
