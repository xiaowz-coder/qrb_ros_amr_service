/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "map_subscriber.hpp"
#include "amr_status_transporter.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <iostream>
#include <fstream>
#include <string>

using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

#define MAP_PATH "map.txt"

namespace qrb_ros
{
namespace amr
{
MapSubscriber::MapSubscriber(
  std::shared_ptr<AMRManager>& amr_manager,
  std::shared_ptr<AMRStatusTransporter>& amr_status_transporter,
  const rclcpp::NodeOptions & options)
  : Node("map_sub",options),
  amr_status_transporter_(amr_status_transporter),
  amr_manager_(amr_manager)
{
  init_subscription();
  read_map();
}

MapSubscriber::~MapSubscriber()
{
}

void MapSubscriber::init_subscription()
{
  using namespace std::placeholders;
  sub_ = create_subscription<OccupancyGrid>(
    "map", 10, std::bind(&MapSubscriber::subscriber_callback, this, _1));
}

void MapSubscriber::subscriber_callback(const OccupancyGrid::SharedPtr map)
{
  if (receive_map_count_ > 1) {
    return;
  }
  RCLCPP_INFO(this->get_logger(), "receive a map info");
  amr_status_transporter_->update_map(*map);
  save_map(*map);
  receive_map_count_++;
}

void MapSubscriber::save_map(OccupancyGrid map)
{
  RCLCPP_INFO(this->get_logger(), "start save file");

  RCLCPP_INFO(this->get_logger(), "finish save file");
}

bool MapSubscriber::read_map()
{

  return false;
}
}  // namespace amr
}  // namespace qrb_ros