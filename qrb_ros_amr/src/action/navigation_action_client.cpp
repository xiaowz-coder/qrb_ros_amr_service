/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "amr_manager.hpp"
#include "navigation_action_client.hpp"
#include "cmd_action_server.hpp"
#include "sub_cmd_service_server.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

constexpr char const * service_name = "follow_path_sub_cmd";

namespace qrb_ros
{
namespace amr
{
NavigationActionClient::NavigationActionClient(
  std::shared_ptr<AMRManager> & amr_manager, const rclcpp::NodeOptions & options)
  : Node("navigator_client", options),
  amr_manager_(amr_manager)
{
  create_nav_client();
}

NavigationActionClient::~NavigationActionClient()
{
}

void NavigationActionClient::create_nav_client()
{
  this->follow_path_client_ptr_ = rclcpp_action::create_client<FollowPath>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "followpath");

  this->waypoint_follow_path_client_ptr_ = rclcpp_action::create_client<WaypointFollowPath>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "wfollowpath");

  this->p2p_client_ptr_ = rclcpp_action::create_client<P2P>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "p2p");

  follow_path_service_client_ = this->create_client<SubCmd>(service_name);

  p2p_nav_callback_ = [&](void* buffer) {
    start_p2p_nav(buffer);
  };
  amr_manager_->register_start_p2p_nav_callback(p2p_nav_callback_);

  follow_path_callback_ = [&](void* path) {
    start_follow_path(path);
  };
  amr_manager_->register_start_follow_path_callback(follow_path_callback_);

  waypoint_follow_path_callback_ = [&](uint32_t goal, std::vector<uint32_t>& ids) {
    start_waypoint_follow_path(goal, ids);
  };
  amr_manager_->register_start_waypoint_follow_path_callback(waypoint_follow_path_callback_);

  sub_cmd_callback_ = [&](bool p2p, uint8_t sub_cmd) {
    handle_sub_cmd(p2p, sub_cmd);
  };
  amr_manager_->register_sub_cmd_callback(sub_cmd_callback_);

  navigate_to_charging_callback_ = [&](void) {
    navigate_to_charging();
  };
  amr_manager_->register_navigate_to_charging_callback(navigate_to_charging_callback_);
}

void NavigationActionClient::start_p2p_nav(void* buffer)
{
  p2p_send_goal(buffer);
}

void NavigationActionClient::start_follow_path(void* path)
{
  follow_send_goal(path);
}

void NavigationActionClient::start_waypoint_follow_path(uint32_t goal, std::vector<uint32_t>& ids)
{
  waypoint_follow_send_goal(goal, ids);
}

void NavigationActionClient::waypoint_follow_send_goal(uint32_t goal, std::vector<uint32_t>& ids)
{
  using namespace std::placeholders;
  auto goal_msg = WaypointFollowPath::Goal();

  goal_msg.goal = goal;
  uint32_t len = ids.size();
  if(len != 0) {
    goal_msg.passing_waypoint_ids.assign(ids.begin(), ids.end());
  }

  RCLCPP_INFO(logger_, "waypoint_follow_send_goal(%d)", len);
  auto send_goal_options = rclcpp_action::Client<WaypointFollowPath>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&NavigationActionClient::waypoint_follow_goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&NavigationActionClient::waypoint_follow_feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&NavigationActionClient::waypoint_follow_result_callback, this, _1);
  this->waypoint_follow_path_client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void NavigationActionClient::follow_send_goal(void* path = NULL)
{
  using namespace std::placeholders;
  auto goal_msg = FollowPath::Goal();
  if (path != NULL){
    temp_path_ = *reinterpret_cast<const Path *>(path);
  }
  goal_msg.path = temp_path_;
  uint32_t len = goal_msg.path.poses.size();
  RCLCPP_INFO(logger_, "follow_path_send_goal(%d)", len);
  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&NavigationActionClient::follow_goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&NavigationActionClient::follow_feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&NavigationActionClient::follow_result_callback, this, _1);
  this->follow_path_client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void NavigationActionClient::p2p_send_goal(void* buffer)
{
  using namespace std::placeholders;
  auto goal_msg = P2P::Goal();
  if (buffer != NULL){
    temp_pose_ = *reinterpret_cast<const PoseStamped *>(buffer);
  }

  double x = temp_pose_.pose.position.x;
  double y = temp_pose_.pose.position.y;
  tf2::Quaternion quat(temp_pose_.pose.orientation.x,
                       temp_pose_.pose.orientation.y,
                       temp_pose_.pose.orientation.z,
                       temp_pose_.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3 m(quat);
  m.getRPY(roll, pitch, yaw);
  double z = yaw;
  RCLCPP_INFO(logger_, "p2p_send_goal(%f,%f,%f)", x, y, z);

  goal_msg.pose = temp_pose_;
  auto send_goal_options = rclcpp_action::Client<P2P>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&NavigationActionClient::p2p_goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&NavigationActionClient::p2p_feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&NavigationActionClient::p2p_result_callback, this, _1);
  this->p2p_client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void NavigationActionClient::handle_sub_cmd(bool p2p, uint8_t sub_cmd)
{
  RCLCPP_INFO(logger_, "handle_sub_cmd, p2p=%d, sub_cmd=%d", p2p, sub_cmd);
  if (p2p) {
    if (sub_cmd == SubCommand::CANCEL) {
      cancel_p2p_navigation();
    } else if (sub_cmd == SubCommand::PAUSE) {
      pause_p2p_navigation();
    } else if (sub_cmd == SubCommand::RESUME) {
      resume_p2p_navigation();
    }
  } else {
    if (sub_cmd == SubCommand::CANCEL) {
      cancel_follow_path();
    } else if (sub_cmd == SubCommand::PAUSE) {
      pause_follow_path();
    } else if (sub_cmd == SubCommand::RESUME) {
      resume_follow_path();
    }
  }
}

void NavigationActionClient::cancel_p2p_navigation()
{
  is_pausing_ = true;
  this->p2p_client_ptr_->async_cancel_all_goals();
}

void NavigationActionClient::cancel_follow_path()
{
  is_pausing_ = true;
  this->waypoint_follow_path_client_ptr_->async_cancel_all_goals();
  this->follow_path_client_ptr_->async_cancel_all_goals();
}

void NavigationActionClient::pause_p2p_navigation()
{
  is_pausing_ = true;
  this->p2p_client_ptr_->async_cancel_all_goals();
}

void NavigationActionClient::pause_follow_path()
{
  RCLCPP_INFO(logger_, "pause_follow_path");
  is_pausing_ = true;

  // get request
  auto request = std::make_shared<SubCmd::Request>();
  request->subcommand = SubCommand::PAUSE;

  if (!wait_for_service()) {
    return;
  }

  // send request
  send_and_wait_for_request(request);
}

void NavigationActionClient::resume_p2p_navigation()
{
  is_pausing_ = false;
  p2p_send_goal();
}

void NavigationActionClient::resume_follow_path()
{
  RCLCPP_INFO(logger_, "resume_follow_path");
  is_pausing_ = false;

  // get request
  auto request = std::make_shared<SubCmd::Request>();
  request->subcommand = SubCommand::RESUME;

  if (!wait_for_service()) {
    return;
  }

  // send request
  send_and_wait_for_request(request);
}

void NavigationActionClient::p2p_goal_response_callback(
    GoalHandleP2P::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(logger_, "Goal was rejected by server");
    CmdActionServer::command_status_completed(false);
    amr_manager_->process_event(Message::CANCEL);
  } else {
    RCLCPP_INFO(logger_, "Goal accepted by server, waiting for result");
    p2p_uuid_ = goal_handle->get_goal_id();
  }
}

void NavigationActionClient::p2p_feedback_callback(
  GoalHandleP2P::SharedPtr,
  const std::shared_ptr<const P2P::Feedback> feedback)
{
  RCLCPP_DEBUG(logger_, "[P2P] Publish feedback");
  CmdActionServer::motion_pose_changed(feedback->current_pose);
}

void NavigationActionClient::p2p_result_callback(const GoalHandleP2P::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
    {
      RCLCPP_INFO(logger_, "Receive P2P result = SUCCEEDED");
      RCLCPP_INFO(logger_, "[P2P] Goal finished!");
      CmdActionServer::command_status_completed(true);
      amr_manager_->process_event(Message::P2PNAV_FINISH);
      return;
    }
    case rclcpp_action::ResultCode::ABORTED:
    {
      RCLCPP_INFO(logger_, "Receive P2P result = ABORTED");
      if (is_pausing_) {
        return;
      }
      RCLCPP_ERROR(logger_, "Goal(p2p) was aborted");
      CmdActionServer::command_status_completed(false);
      amr_manager_->process_event(Message::CANCEL);
      return;
    }
    case rclcpp_action::ResultCode::CANCELED:
    {
      RCLCPP_INFO(logger_, "Receive P2P result = CANCELED");
      if (is_pausing_) {
        return;
      }
      RCLCPP_ERROR(logger_, "Goal(p2p) was canceled");
      CmdActionServer::command_status_completed(false);
      amr_manager_->process_event(Message::CANCEL);
      return;
    }
    default:
      RCLCPP_ERROR(logger_, "Unknown result code");
      return;
  }
}

void NavigationActionClient::follow_goal_response_callback(
    GoalHandleFollow::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(logger_, "Goal was rejected by server");
    CmdActionServer::command_status_completed(false);
    amr_manager_->process_event(Message::CANCEL);
  } else {
    RCLCPP_INFO(logger_, "Goal accepted by server, waiting for result");
    follow_uuid_ = goal_handle->get_goal_id();
  }
}

void NavigationActionClient::follow_feedback_callback(
  GoalHandleFollow::SharedPtr,
  const std::shared_ptr<const FollowPath::Feedback> feedback)
{
  RCLCPP_DEBUG(logger_, "[FOLLOW_PATH] Publish feedback");
}

void NavigationActionClient::follow_result_callback(const GoalHandleFollow::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
    {
      RCLCPP_INFO(logger_, "Receive follow path result = SUCCEEDED");
      RCLCPP_INFO(logger_, "[FOLLOW_PATH] Goal finished!");
      CmdActionServer::command_status_completed(true);
      amr_manager_->process_event(Message::FOLLOW_PATH_FINISH);
      return;
    }
    case rclcpp_action::ResultCode::ABORTED:
    {
      RCLCPP_INFO(logger_, "Receive follow path result = ABORTED");
      if (is_pausing_) {
        return;
      }
      RCLCPP_ERROR(logger_, "Goal was aborted");
      CmdActionServer::command_status_completed(false);
      amr_manager_->process_event(Message::CANCEL);
      return;
    }
    case rclcpp_action::ResultCode::CANCELED:
    {
      RCLCPP_INFO(logger_, "Receive follow path result = CANCELED");
      if (is_pausing_) {
        return;
      }
      RCLCPP_ERROR(logger_, "Goal(follow path) was canceled");
      CmdActionServer::command_status_completed(false);
      amr_manager_->process_event(Message::CANCEL);
      return;
    }
    default:
      RCLCPP_ERROR(logger_, "Unknown result code");
      return;
  }
}

void NavigationActionClient::waypoint_follow_goal_response_callback(
    GoalHandleWaypointFollowPath::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(logger_, "Goal was rejected by server");
    CmdActionServer::command_status_completed(false);
    amr_manager_->process_event(Message::CANCEL);
  } else {
    RCLCPP_INFO(logger_, "Goal accepted by server, waiting for result");
    follow_uuid_ = goal_handle->get_goal_id();
  }
}

void NavigationActionClient::waypoint_follow_feedback_callback(
  GoalHandleWaypointFollowPath::SharedPtr,
  const std::shared_ptr<const WaypointFollowPath::Feedback> feedback)
{
  uint32_t passing_waypoint_id = feedback->passing_waypoint_id;
  RCLCPP_INFO(logger_, "[WAYPOINT_FOLLOW_PATH] Publish feedback, passing_waypoint_id = %d", passing_waypoint_id);
}

void NavigationActionClient::waypoint_follow_result_callback(
  const GoalHandleWaypointFollowPath::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
    {
      RCLCPP_INFO(logger_, "[WAYPOINT_FOLLOW_PATH] Goal finished!");
      CmdActionServer::command_status_completed(true);
      amr_manager_->process_event(Message::FOLLOW_PATH_FINISH);
      return;
    }
    case rclcpp_action::ResultCode::ABORTED:
    {
      if (is_pausing_) {
        return;
      }
      RCLCPP_ERROR(logger_, "Goal was aborted");
      CmdActionServer::command_status_completed(false);
      amr_manager_->process_event(Message::CANCEL);
      return;
    }
    case rclcpp_action::ResultCode::CANCELED:
    {
      RCLCPP_INFO(logger_, "Receive wfollow path result = CANCELED");
      if (is_pausing_) {
        return;
      }
      RCLCPP_ERROR(logger_, "Goal(wfollow path) was canceled");
      CmdActionServer::command_status_completed(false);
      amr_manager_->process_event(Message::CANCEL);
      return;
    }
    default:
      RCLCPP_ERROR(logger_, "Unknown result code");
      return;
  }
}

void NavigationActionClient::navigate_to_charging()
{
  using namespace std::placeholders;

  auto goal = P2P::Goal();
  double charging_station_x = 0.3;
  double charging_station_y = 0;
  double charging_station_angle = 0;

  generate_pose(charging_station_x, charging_station_y, charging_station_angle, charging_station_);

  RCLCPP_INFO(logger_, "Return charger station(%f,%f,%f)", charging_station_x, charging_station_y, charging_station_angle);

  goal.pose = charging_station_;
  auto send_goal_options = rclcpp_action::Client<P2P>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&NavigationActionClient::charging_station_goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&NavigationActionClient::charging_station_feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&NavigationActionClient::charging_station_result_callback, this, _1);
  this->p2p_client_ptr_->async_send_goal(goal, send_goal_options);

  std::unique_lock<std::mutex> lck(mtx_);
  cv_.wait(lck);
}

void NavigationActionClient::charging_station_goal_response_callback(
    GoalHandleP2P::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(logger_, "Goal of going charging station was rejected by server");
    cv_.notify_one();
  } else {
    RCLCPP_INFO(logger_, "Goal of going charging station accepted by server, waiting for result");
  }
}

void NavigationActionClient::charging_station_feedback_callback(
  GoalHandleP2P::SharedPtr,
  const std::shared_ptr<const P2P::Feedback> feedback)
{
  RCLCPP_DEBUG(logger_, "[Charging station] Publish feedback");
  CmdActionServer::motion_pose_changed(feedback->current_pose);
}

void NavigationActionClient::charging_station_result_callback(const GoalHandleP2P::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(logger_, "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(logger_, "Goal(charging) was canceled");
      break;
    default:
      RCLCPP_ERROR(logger_, "Unknown result code");
      break;
  }
  RCLCPP_INFO(logger_, "[Charging station] Goal finished!");
  cv_.notify_one();
  return;
}

void NavigationActionClient::generate_pose(
  double& x, double& y, double& z, PoseStamped& pose)
{
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0;

  tf2::Quaternion quat;
  quat.setRPY(0, 0, z);
  pose.pose.orientation.x = quat.x();
  pose.pose.orientation.y = quat.y();
  pose.pose.orientation.z = quat.z();
  pose.pose.orientation.w = quat.w();
}

bool NavigationActionClient::wait_for_service()
{
  uint32_t wait = 0;
  // wait for connectting service
  while (!follow_path_service_client_->wait_for_service(std::chrono::seconds(1))) {
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

bool NavigationActionClient::send_and_wait_for_request(SubCmd::Request::SharedPtr request)
{
  RCLCPP_INFO(logger_, "send_and_wait_for_request");
  using ServiceResponseFuture = rclcpp::Client<SubCmd>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
    std::unique_lock<std::mutex> lck(mtx_);
    auto result = future.get();
    response_result_ = result->result;
    cv_.notify_one();
  };

  auto future_result = follow_path_service_client_->async_send_request(request, response_received_callback);
  std::unique_lock<std::mutex> lck(mtx_);
  cv_.wait(lck);

  if (response_result_) {
    RCLCPP_INFO(logger_, "api is excuted successful");
  } else {
    RCLCPP_ERROR(logger_, "api is excuted failed");
  }
  return response_result_;
}
}  // namespace amr
}  // namespace qrb_ros
