/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_AMR__NAVIGATION_ACTION_CLIENT_HPP_
#define QRB_ROS_AMR__NAVIGATION_ACTION_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "qrb_ros_navigation_msgs/action/follow_path.hpp"
#include "qrb_ros_amr_msgs/srv/sub_cmd.hpp"
#include "nav_msgs/msg/path.hpp"
#include "amr_manager.hpp"
#include <memory>

using PoseStamped = geometry_msgs::msg::PoseStamped;
using Path = nav_msgs::msg::Path;
using FollowPath = nav2_msgs::action::FollowPath;
using WaypointFollowPath = qrb_ros_navigation_msgs::action::FollowPath;
using P2P = nav2_msgs::action::NavigateToPose;
using GoalHandleFollow = rclcpp_action::ClientGoalHandle<FollowPath>;
using GoalHandleWaypointFollowPath = rclcpp_action::ClientGoalHandle<WaypointFollowPath>;
using GoalHandleP2P = rclcpp_action::ClientGoalHandle<P2P>;

using SubCmd = qrb_ros_amr_msgs::srv::SubCmd;
using namespace qrb::amr_manager;

#define SERVICE_TIMEOUT_DURATION 5  // timeout is 5 seconds

namespace qrb_ros
{
namespace amr
{

class NavigationActionClient : public rclcpp::Node
{
private:
  PoseStamped temp_pose_;
  Path temp_path_;
  PoseStamped charging_station_;
  std::shared_ptr<AMRManager> amr_manager_;

  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_ptr_;
  rclcpp_action::Client<WaypointFollowPath>::SharedPtr waypoint_follow_path_client_ptr_;
  rclcpp_action::Client<P2P>::SharedPtr p2p_client_ptr_;
  rclcpp_action::GoalUUID p2p_uuid_;
  rclcpp_action::GoalUUID follow_uuid_;
  bool is_pausing_ = false;
  bool is_return_charging_ = false;
  std::mutex mtx_;
  std::condition_variable cv_;

  std::shared_ptr<rclcpp::Client<SubCmd>> follow_path_service_client_;
  bool response_result_;
  qrb::amr_manager::start_p2p_func_t p2p_nav_callback_;
  qrb::amr_manager::start_follow_path_func_t follow_path_callback_;
  qrb::amr_manager::start_waypoint_follow_path_func_t waypoint_follow_path_callback_;
  qrb::amr_manager::sub_cmd_func_t sub_cmd_callback_;
  qrb::amr_manager::navigate_to_charging_func_t navigate_to_charging_callback_;
  rclcpp::Logger logger_{ rclcpp::get_logger("navigation_action_client") };

  void create_nav_client();
  void p2p_send_goal(void * buffer = NULL);
  void follow_send_goal(void * path);
  void waypoint_follow_send_goal(uint32_t goal, std::vector<uint32_t> & ids);

  void p2p_goal_response_callback(GoalHandleP2P::SharedPtr goal_handle);
  void p2p_feedback_callback(GoalHandleP2P::SharedPtr,
      const std::shared_ptr<const P2P::Feedback> feedback);
  void p2p_result_callback(const GoalHandleP2P::WrappedResult & result);
  void follow_goal_response_callback(GoalHandleFollow::SharedPtr goal_handle);
  void follow_feedback_callback(GoalHandleFollow::SharedPtr,
      const std::shared_ptr<const FollowPath::Feedback> feedback);
  void follow_result_callback(const GoalHandleFollow::WrappedResult & result);
  void waypoint_follow_goal_response_callback(GoalHandleWaypointFollowPath::SharedPtr goal_handle);
  void waypoint_follow_feedback_callback(GoalHandleWaypointFollowPath::SharedPtr,
      const std::shared_ptr<const WaypointFollowPath::Feedback> feedback);
  void waypoint_follow_result_callback(const GoalHandleWaypointFollowPath::WrappedResult & result);

  void charging_station_goal_response_callback(GoalHandleP2P::SharedPtr goal_handle);
  void charging_station_feedback_callback(GoalHandleP2P::SharedPtr,
      const std::shared_ptr<const P2P::Feedback> feedback);
  void charging_station_result_callback(const GoalHandleP2P::WrappedResult & result);
  void generate_pose(double & x, double & y, double & z, PoseStamped & pose);
  void handle_sub_cmd(bool p2p, uint8_t sub_cmd);

  bool wait_for_service();
  bool send_and_wait_for_request(SubCmd::Request::SharedPtr request);

public:
  NavigationActionClient(std::shared_ptr<AMRManager> & amr_manager,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~NavigationActionClient();
  void start_p2p_nav(void * buffer);
  void start_follow_path(void * path);
  void start_waypoint_follow_path(uint32_t goal, std::vector<uint32_t> & ids);
  void cancel_p2p_navigation();
  void cancel_follow_path();
  void pause_p2p_navigation();
  void pause_follow_path();
  void resume_p2p_navigation();
  void resume_follow_path();
  void navigate_to_charging();
};
}  // namespace amr
}  // namespace qrb_ros
#endif  // QRB_ROS_AMR__NAVIGATION_ACTION_CLIENT_HPP_