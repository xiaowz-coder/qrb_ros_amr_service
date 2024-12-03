/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef AMR_CONTROLLER_HPP_
#define AMR_CONTROLLER_HPP_

#include "api_service_server.hpp"
#include "mapping_service_server.hpp"
#include "cmd_action_server.hpp"
#include "sub_cmd_service_server.hpp"
#include "amr_status_transporter.hpp"
#include "auto_mapper_action_client.hpp"
#include "map_subscriber.hpp"
#include "exception_subscriber.hpp"
#include "developer_mode_subscriber.hpp"
#include "navigation_action_client.hpp"
#include "amr_mode_service_client.hpp"
#include "cartographer_service_client.hpp"
#include "amr_manager.hpp"

using namespace qrb::amr_manager;

namespace qrb_ros
{
namespace amr
{
class APIServiceServer;

class AMRController
{
private:
  std::shared_ptr<AMRManager> amr_manager_;
  std::shared_ptr<APIServiceServer> api_server_;
  std::shared_ptr<MappingServiceServer> mapping_server_;
  std::shared_ptr<DeveloperModeSubscriber> dev_sub_;
  std::shared_ptr<CmdActionServer> cmd_server_;
  std::shared_ptr<SubCmdServer> sub_cmd_server_;
  std::shared_ptr<AMRStatusTransporter> amr_status_transporter_;
  std::shared_ptr<AutoMapperActionClient> auto_mapper_client_;
  std::shared_ptr<NavigationActionClient> nav_client_;
  std::shared_ptr<ExceptionSubscriber> exception_sub_;
  std::shared_ptr<LowPowerManager> low_power_;
  std::shared_ptr<CartographerServiceClient> cartographer_service_client_;
  std::shared_ptr<AMRModeServiceClient> amr_mode_client_;

  void init_nodes();

  rclcpp::Logger logger_{rclcpp::get_logger("amr_controller")};

public:
  AMRController();
  ~AMRController();

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
};
}  // namespace amr
}  // namespace qrb_ros
#endif // AMR_CONTROLLER_HPP_