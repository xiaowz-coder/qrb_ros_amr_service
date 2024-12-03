/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef SUB_CMD_SERVICE_SERVER_HPP_
#define SUB_CMD_SERVICE_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "qrb_ros_amr_msgs/srv/sub_cmd.hpp"
#include "amr_manager.hpp"
#include <mutex>
#include <condition_variable>
#include <memory>
#include <string>

using namespace qrb::amr_manager;

namespace qrb_ros
{
namespace amr
{

class SubCmdServer : public rclcpp::Node
{
private:
  rclcpp::Service<qrb_ros_amr_msgs::srv::SubCmd>::SharedPtr srv_ptr_;
  std::shared_ptr<AMRManager> amr_manager_;

  void init_service();
  void handle_sub_cmd(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<qrb_ros_amr_msgs::srv::SubCmd::Request> request,
    std::shared_ptr<qrb_ros_amr_msgs::srv::SubCmd::Response> response);

public:
  SubCmdServer(
      std::shared_ptr<AMRManager> & amr_manager,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SubCmdServer();
};
}  // namespace amr
}  // namespace qrb_ros

#endif // SUB_CMD_SERVICE_SERVER_HPP_