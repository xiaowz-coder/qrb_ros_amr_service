/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_AMR__MAP_SUBSCRIBER_HPP_
#define QRB_ROS_AMR__MAP_SUBSCRIBER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "amr_status_transporter.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "amr_manager.hpp"
#include <memory>

using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

using namespace qrb::amr_manager;

namespace qrb_ros
{
namespace amr
{

class MapSubscriber : public rclcpp::Node
{
private:
  rclcpp::Subscription<OccupancyGrid>::SharedPtr sub_;
  std::shared_ptr<AMRStatusTransporter> amr_status_transporter_;
  std::shared_ptr<AMRManager> amr_manager_;

  void init_subscription();
  bool read_map();
  void save_map(OccupancyGrid map);
  int receive_map_count_ = 0;

public:
  MapSubscriber(std::shared_ptr<AMRManager> & amr_manager,
      std::shared_ptr<AMRStatusTransporter> & amr_status_transporter,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MapSubscriber();
  void subscriber_callback(const OccupancyGrid::SharedPtr map);
};
}  // namespace amr
}  // namespace qrb_ros
#endif  // QRB_ROS_AMR__MAP_SUBSCRIBER_HPP_