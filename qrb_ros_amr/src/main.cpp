/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "amr_controller.hpp"
#include <memory>

using namespace std;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  qrb_ros::amr::AMRController control;
  (control.executor_)->spin();
  rclcpp::shutdown();
  return 0;
}