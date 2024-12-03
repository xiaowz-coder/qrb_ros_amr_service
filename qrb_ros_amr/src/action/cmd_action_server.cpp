/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "qrb_ros_amr_msgs/action/cmd.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cmd_action_server.hpp"
#include <iostream>
#include <memory>

namespace qrb_ros
{
namespace amr
{
std::shared_ptr<GoalHandleCmd> CmdActionServer::record_goal_ = NULL;

CmdActionServer::CmdActionServer(std::shared_ptr<AMRManager>& amr_manager,
  const rclcpp::NodeOptions & options)
: Node("cmd_action_server", options),
  amr_manager_(amr_manager)
{
  create_server();
}

CmdActionServer::~CmdActionServer()
{
  record_goal_ = nullptr;
}

void CmdActionServer::motion_pose_changed(Pose pose)
{
  using namespace std;
  auto feedback = std::make_shared<Cmd::Feedback>();
  feedback->current_pose = pose;
  if (record_goal_ == nullptr) {
    //navigator to charging_station will run into here
    return;
  }
  record_goal_->publish_feedback(feedback);
  RCLCPP_DEBUG(rclcpp::get_logger("cmd_action_server"), "Publish feedback");
}

void CmdActionServer::command_status_completed(bool cmdStatus)
{
  using namespace std;
  auto finish = std::make_shared<Cmd::Result>();
  finish->result = cmdStatus;
  if (record_goal_ == nullptr) {
    RCLCPP_INFO(rclcpp::get_logger("cmd_action_server"), "server global handle is nullptr");
    return;
  }

  if (!cmdStatus) {
    if (record_goal_->is_canceling()) {
      record_goal_->canceled(finish);
      RCLCPP_INFO(rclcpp::get_logger("cmd_action_server"), "Goal canceled");
    } else {
      record_goal_->abort(finish);
      RCLCPP_INFO(rclcpp::get_logger("cmd_action_server"), "Goal abort");
    }
    return;
  }

  record_goal_->succeed(finish);
  record_goal_ = nullptr;
  RCLCPP_INFO(rclcpp::get_logger("cmd_action_server"), "Goal finished!");
}

bool CmdActionServer::is_waiting_command_feedback()
{
  return is_waiting_command_;
}

void CmdActionServer::create_server()
{
  using namespace std::placeholders;
  this->server_ptr_ = rclcpp_action::create_server<Cmd>(
  this->get_node_base_interface(),
  this->get_node_clock_interface(),
  this->get_node_logging_interface(),
  this->get_node_waitables_interface(),
  "cmd",
  std::bind(&CmdActionServer::handle_goal, this, _1, _2),
  std::bind(&CmdActionServer::handle_cancel, this, _1),
  std::bind(&CmdActionServer::handle_accepted, this, _1));
}

void CmdActionServer::set_waiting_command_flag(bool flag)
{
  is_waiting_command_ = flag;
}

rclcpp_action::GoalResponse CmdActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Cmd::Goal> goal)
{
  (void) uuid;
  RCLCPP_INFO(logger_, "Received request to handle goal");
  int cmd = goal->command;
  bool succeed = amr_manager_->check_potential_state(cmd);
  if (succeed) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse CmdActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleCmd> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO(logger_, "Received request to cancel goal: %s",
             (Command::cmd_to_string(goal->command)).c_str());
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CmdActionServer::handle_accepted(const std::shared_ptr<GoalHandleCmd> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&CmdActionServer::execute, this, _1), goal_handle}.detach();
}

void CmdActionServer::execute(const std::shared_ptr<GoalHandleCmd> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO(logger_, "execute goal : %s", (Command::cmd_to_string(goal->command)).c_str());
  void* ptr;
  if (goal->command == Command::AE) {
    amr_manager_->process_cmd(Message::AE, nullptr, 0);
  } else if (goal->command == Command::ME) {
    amr_manager_->process_cmd(Message::ME, nullptr, 0);
  } else if (goal->command == Command::P2PNAV) {
    ptr = (void*) &(goal->goal);
    RCLCPP_INFO(logger_, "execute goal size: %d", sizeof(goal->goal));
    amr_manager_->process_cmd(Message::P2PNAV, ptr, sizeof(goal->goal));
  } else if (goal->command == Command::FOLLOW_PATH) {
    ptr = (void*) &(goal->path);
    RCLCPP_INFO(logger_, "execute goal size: %d", sizeof(goal->goal));
    amr_manager_->process_cmd(Message::FOLLOW_PATH, ptr, sizeof(goal->path));
  } else if (goal->command == Command::CHARGING) {
    amr_manager_->process_cmd(Message::RETURN_CHARGING, nullptr, 0);
  } else if (goal->command == Command::WaypointFollowPath) {
    uint32_t goal_id = goal->goal_id;
    std::vector<uint32_t> ids;
    if(goal->passing_waypoint_ids.size() != 0) {
      ids.assign(goal->passing_waypoint_ids.begin(), goal->passing_waypoint_ids.end());
    }
    RCLCPP_INFO(logger_, "execute goal id =%d, passing_waypoint size: %d",
                goal_id, sizeof(goal->passing_waypoint_ids));
    amr_manager_->process_cmd(Message::WAYPOINT_FOLLOW_PATH, goal_id, ids);
  }
  record_goal_ = goal_handle;
}
}  // namespace amr
}  // namespace qrb_ros