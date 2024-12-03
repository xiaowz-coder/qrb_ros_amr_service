/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef AUTO_MAPPER_ACTION_CLIENT_HPP_
#define AUTO_MAPPER_ACTION_CLIENT_HPP_

#include "rclcpp_action/rclcpp_action.hpp"
#include "qrb_ros_amr_msgs/action/ae.hpp"
#include "amr_manager.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <memory>

using AE = qrb_ros_amr_msgs::action::AE;
using GoalHandleAE = rclcpp_action::ClientGoalHandle<AE>;
using Pose = geometry_msgs::msg::PoseStamped;

using namespace qrb::amr_manager;

namespace qrb_ros
{
namespace amr
{

class AutoMapperActionClient : public rclcpp::Node
{
private:
  rclcpp_action::Client<AE>::SharedPtr client_ptr_;
  std::shared_ptr<AMRManager> amr_manager_;

  void create_client();
  void send_goal();
  void goal_response_callback(GoalHandleAE::SharedPtr goal_handle);
  void feedback_callback(GoalHandleAE::SharedPtr,
    const std::shared_ptr<const AE::Feedback> feedback);
  void result_callback(const GoalHandleAE::WrappedResult & result);

public:
  AutoMapperActionClient(
      std::shared_ptr<AMRManager> & amr_manager,
	  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AutoMapperActionClient();
  void start_ae();
  void cancel_goal();
};
}  // namespace amr
}  // namespace qrb_ros
#endif // AUTO_MAPPER_ACTION_CLIENT_HPP_