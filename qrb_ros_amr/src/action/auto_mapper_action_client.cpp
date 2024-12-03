/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "amr_manager.hpp"
#include "auto_mapper_action_client.hpp"
#include "cmd_action_server.hpp"

namespace qrb_ros
{
namespace amr
{
AutoMapperActionClient::AutoMapperActionClient(
  std::shared_ptr<AMRManager>& amr_manager, const rclcpp::NodeOptions& options)
  : Node("auto_mapper_client", options),
  amr_manager_(amr_manager)
{
  create_client();
}

AutoMapperActionClient::~AutoMapperActionClient()
{
}

void AutoMapperActionClient::start_ae()
{
  send_goal();
}

void AutoMapperActionClient::cancel_goal()
{
  RCLCPP_INFO(this->get_logger(), "cancel all goal");
  this->client_ptr_->async_cancel_all_goals();
}

void AutoMapperActionClient::create_client()
{
  this->client_ptr_ = rclcpp_action::create_client<AE>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "ae");
}

void AutoMapperActionClient::send_goal()
{
  using namespace std::placeholders;
  auto goal_msg = AE::Goal();
  auto send_goal_options = rclcpp_action::Client<AE>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&AutoMapperActionClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&AutoMapperActionClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&AutoMapperActionClient::result_callback, this, _1);
  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void AutoMapperActionClient::goal_response_callback(
       GoalHandleAE::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    CmdActionServer::command_status_completed(false);
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void AutoMapperActionClient::feedback_callback(GoalHandleAE::SharedPtr,
  const std::shared_ptr<const AE::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Publish feedback!");
  CmdActionServer::motion_pose_changed(feedback->current_pose);
}

void AutoMapperActionClient::result_callback(const GoalHandleAE::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      CmdActionServer::command_status_completed(false);
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      CmdActionServer::command_status_completed(false);
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }
  RCLCPP_INFO(this->get_logger(), "Goal finished!");
  CmdActionServer::command_status_completed(true);
  amr_manager_->process_event(Message::AE_FINISH);
}

}  // namespace amr
}  // namespace qrb_ros