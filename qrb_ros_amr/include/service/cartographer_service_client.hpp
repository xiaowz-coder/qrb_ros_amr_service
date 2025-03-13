/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_AMR__CARTOGRAPHER_SERVICE_CLIENT_HPP_
#define QRB_ROS_AMR__CARTOGRAPHER_SERVICE_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "amr_manager.hpp"
#include "qrb_ros_slam_msgs/srv/slam_command.hpp"
#include "qrb_ros_slam_msgs/msg/command_code.hpp"
#include "qrb_ros_navigation_msgs/srv/virtual_path.hpp"

using namespace qrb::amr_manager;
using SlamCommand = qrb_ros_slam_msgs::srv::SlamCommand;
using CommandCode = qrb_ros_slam_msgs::msg::CommandCode;
using VirtualPath = qrb_ros_navigation_msgs::srv::VirtualPath;

#define SERVICE_TIMEOUT_DURATION 5  // timeout is 5 seconds
#define RemoveWaypointAndVirtualPath 29

namespace qrb_ros
{
namespace amr
{

enum class Slam_Command
{
  StartMapping = 1,
  StopMapping = 2,
  SaveMap = 3,
  LoadMap = 4,
  StartLocalization = 5,
  Relocalization = 6,
};

/**
 * @class amr_controller::CartographerServiceClientNode
 * @desc The CartographerServiceClientNode create service client to send requests to
 * notify cartographer.
 */
class CartographerServiceClient : public rclcpp::Node
{
public:
  CartographerServiceClient(std::shared_ptr<AMRManager> & amr_manager);
  ~CartographerServiceClient();

private:
  bool send_api_request(SlamCommand::Request::SharedPtr request);
  bool wait_for_service();
  bool send_and_wait_for_request(SlamCommand::Request::SharedPtr request);
  bool send_api_request2(VirtualPath::Request::SharedPtr request);
  bool wait_for_service2();
  bool send_and_wait_for_response2(VirtualPath::Request::SharedPtr request);

  void init_client();
  bool handle_slam_command(uint8_t cmd);
  bool start_mapping();
  bool stop_mapping();
  bool save_map();
  bool load_map();
  bool start_localization();
  bool relocalization();
  bool remove_waypoint_and_virtual_path();

  std::shared_ptr<rclcpp::Client<SlamCommand>> client_;
  std::shared_ptr<rclcpp::Client<VirtualPath>> virtual_path_client_;
  std::shared_ptr<AMRManager> amr_manager_;
  std::mutex mtx_;
  std::condition_variable cv_;
  bool response_result_;
  uint8_t virtual_path_response_result_;
  slam_command_func_t slam_command_callback_;
  rclcpp::Logger logger_{ rclcpp::get_logger("cartographer_service_client") };
};

}  // namespace amr
}  // namespace qrb_ros
#endif  // QRB_ROS_AMR__CARTOGRAPHER_SERVICE_CLIENT_HPP_
