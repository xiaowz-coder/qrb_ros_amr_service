/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_AMR__CMD_ACTION_SERVER_HPP_
#define QRB_ROS_AMR__CMD_ACTION_SERVER_HPP_

#include "rclcpp_action/rclcpp_action.hpp"
#include "qrb_ros_amr_msgs/action/cmd.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "amr_manager.hpp"
#include <memory>
#include <string>

using Pose = geometry_msgs::msg::PoseStamped;
using Cmd = qrb_ros_amr_msgs::action::Cmd;
using GoalHandleCmd = rclcpp_action::ServerGoalHandle<Cmd>;

using namespace qrb::amr_manager;

namespace qrb_ros
{
namespace amr
{

class Command
{
public:
  const static int ME = 1;
  const static int AE = 2;
  const static int P2PNAV = 3;
  const static int FOLLOW_PATH = 4;
  const static int CHARGING = 5;
  const static int WaypointFollowPath = 6;
  const static int SUB_CMD = 7;
  const static int OTHER = 8;

  static std::string cmd_to_string(int cmd)
  {
    std::string message;
    switch (cmd) {
      case ME:
        message = "ME";
        break;
      case AE:
        message = "AE";
        break;
      case P2PNAV:
        message = "P2PNAV";
        break;
      case FOLLOW_PATH:
        message = "FOLLOW_PATH";
        break;
      case CHARGING:
        message = "CHARGING";
        break;
      case SUB_CMD:
        message = "SUB_CMD";
        break;
      case OTHER:
        message = "OTHER";
        break;
      default:
        message = "INVAILD";
        break;
    }
    return message;
  }
};

class CmdActionServer : public rclcpp::Node
{
private:
  rclcpp_action::Server<qrb_ros_amr_msgs::action::Cmd>::SharedPtr server_ptr_;
  std::shared_ptr<AMRManager> amr_manager_;
  bool is_waiting_command_;
  void create_server();
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const Cmd::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCmd> goal_handle);
  static std::shared_ptr<GoalHandleCmd> record_goal_;

  void handle_accepted(const std::shared_ptr<GoalHandleCmd> goal_handle);
  void execute(const std::shared_ptr<GoalHandleCmd> goal_handle);
  void set_waiting_command_flag(bool flag);

  rclcpp::Logger logger_{ rclcpp::get_logger("cmd_action_server") };

public:
  CmdActionServer(std::shared_ptr<AMRManager> & amr_manager,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CmdActionServer();
  bool is_waiting_command_feedback();
  static void command_status_completed(bool cmd_status);
  static void motion_pose_changed(Pose pose);
};
}  // namespace amr
}  // namespace qrb_ros
#endif  // QRB_ROS_AMR__CMD_ACTION_SERVER_HPP_