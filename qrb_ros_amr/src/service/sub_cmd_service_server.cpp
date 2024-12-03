/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "rclcpp/rclcpp.hpp"
#include "qrb_ros_amr_msgs/srv/sub_cmd.hpp"
#include "sub_cmd_service_server.hpp"
#include "cmd_action_server.hpp"
#include "amr_manager.hpp"
#include <mutex>
#include <condition_variable>

namespace qrb_ros
{
namespace amr
{
SubCmdServer::SubCmdServer(
  std::shared_ptr<AMRManager>& amr_manager,
  const rclcpp::NodeOptions& options)
: Node("sub_cmd_server", options),
  amr_manager_(amr_manager)
{
  init_service();
}

SubCmdServer::~SubCmdServer()
{
}

void SubCmdServer::init_service()
{
  using namespace std::placeholders;
  srv_ptr_ = this->create_service<qrb_ros_amr_msgs::srv::SubCmd>(
          "sub_cmd", std::bind(&SubCmdServer::handle_sub_cmd, this, _1, _2, _3));
}

void SubCmdServer::handle_sub_cmd(const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<qrb_ros_amr_msgs::srv::SubCmd::Request> request,
  std::shared_ptr<qrb_ros_amr_msgs::srv::SubCmd::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "handle_sub_cmd");
  (void)request_header;
  int cmd = request->subcommand;
  RCLCPP_INFO(this->get_logger(),
      "Incoming request sub cmd: %s", (SubCommand::to_string(cmd)).c_str());
  bool succeed = amr_manager_->check_potential_state(Command::SUB_CMD);
  if (!succeed) {
    RCLCPP_INFO(this->get_logger(), "current state is not ON_Nav or ON_FollowPath");
    response->result = false;
    return;
  }

  int msg;
  if (cmd == SubCommand::CANCEL) {
    msg = Message::CANCEL;
  } else if (cmd == SubCommand::PAUSE) {
    msg = Message::PAUSE;
  } else if (cmd == SubCommand::RESUME) {
    msg = Message::RESUME;
  } else {
    RCLCPP_INFO(this->get_logger(), "error: unkown sub command");
    response->result = false;
    return;
  }
  amr_manager_->process_sub_cmd(msg);
  response->result = true;
}
}  // namespace amr
}  // namespace qrb_ros