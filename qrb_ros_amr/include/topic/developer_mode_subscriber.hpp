/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_AMR__DEVELOPER_MODE_SUBSCRIBER_HPP_
#define QRB_ROS_AMR__DEVELOPER_MODE_SUBSCRIBER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "amr_manager.hpp"
#include <memory>
#include "exception_subscriber.hpp"
#include "amr_status_transporter.hpp"

using namespace qrb::amr_manager;

namespace qrb_ros
{
namespace amr
{
/**
 * @enum amr_controller::DEBUG_MSG
 * @desc Enum class representing debug message.
 */
enum class DEBUG_MSG
{
  Init_AMR = 1,
  Release_AMR = 2,
  ME = 3,
  AE = 4,
  ME_Finish = 5,
  AE_Finish = 6,
  ME_Completed = 7,
  P2PNav = 8,
  FollowPath = 9,
  Pause = 10,
  Resume = 11,
  Cancel = 12,
  P2PNav_Finish = 13,
  FollowPath_Finish = 14,
  Low_Power = 15,
  Normal_Power = 16,
  Return_Charging = 17,
  Return_Charging_Finish = 18,
  AMR_Exception = 19,
  AMR_Normal = 20,
  Relocalization_Pass = 21,
};

class StringUtil
{
public:
  static std::string msg_to_string(int cmd)
  {
    std::string str;
    switch ((DEBUG_MSG)cmd) {
      case DEBUG_MSG::Init_AMR:
        str = "Init_AMR";
        break;
      case DEBUG_MSG::Release_AMR:
        str = "Release_AMR";
        break;
      case DEBUG_MSG::ME:
        str = "ME";
        break;
      case DEBUG_MSG::AE:
        str = "AE";
        break;
      case DEBUG_MSG::ME_Finish:
        str = "ME_Finish";
        break;
      case DEBUG_MSG::AE_Finish:
        str = "AE_Finish";
        break;
      case DEBUG_MSG::ME_Completed:
        str = "ME_Completed";
        break;
      case DEBUG_MSG::P2PNav:
        str = "P2PNav";
        break;
      case DEBUG_MSG::FollowPath:
        str = "FollowPath";
        break;
      case DEBUG_MSG::Pause:
        str = "Pause";
        break;
      case DEBUG_MSG::Resume:
        str = "Resume";
        break;
      case DEBUG_MSG::Cancel:
        str = "Cancel";
        break;
      case DEBUG_MSG::P2PNav_Finish:
        str = "P2PNav_Finish";
        break;
      case DEBUG_MSG::FollowPath_Finish:
        str = "FollowPath_Finish";
        break;
      case DEBUG_MSG::Low_Power:
        str = "Low_Power";
        break;
      case DEBUG_MSG::Normal_Power:
        str = "Normal_Power";
        break;
      case DEBUG_MSG::Return_Charging:
        str = "Return_Charging";
        break;
      case DEBUG_MSG::Return_Charging_Finish:
        str = "Return_Charging_Finish";
        break;
      case DEBUG_MSG::AMR_Exception:
        str = "AMR_Exception";
        break;
      case DEBUG_MSG::AMR_Normal:
        str = "AMR_Normal";
        break;
      case DEBUG_MSG::Relocalization_Pass:
        str = "Relocalization_Pass";
        break;
      default:
        str = "Not support debug command";
        break;
    }
    return str;
  }
};

class DeveloperModeSubscriber : public rclcpp::Node
{
private:
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr debug_error_sub_;
  std::shared_ptr<ExceptionSubscriber> exception_sub_;
  std::shared_ptr<AMRStatusTransporter> tansporter_;
  std::shared_ptr<AMRManager> amr_manager_;
  bool enable_developer_mode_;
  rclcpp::Logger logger_{ rclcpp::get_logger("developer_mode_subscriber") };

  void sub_callback(const std_msgs::msg::Int16::ConstSharedPtr msg);
  void debug_error_sub_callback(const std_msgs::msg::Int16::ConstSharedPtr msg);
  void init_subscription();
  void send_debug_message(int event);

public:
  DeveloperModeSubscriber(std::shared_ptr<AMRManager> & amr_manager,
      std::shared_ptr<ExceptionSubscriber> & exception_sub,
      std::shared_ptr<AMRStatusTransporter> & tansporter,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DeveloperModeSubscriber();

  void set_developer_mode(bool val);
};
}  // namespace amr
}  // namespace qrb_ros
#endif  // QRB_ROS_AMR__DEVELOPER_MODE_SUBSCRIBER_HPP_