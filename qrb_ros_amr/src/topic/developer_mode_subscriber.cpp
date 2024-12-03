/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "developer_mode_subscriber.hpp"

namespace qrb_ros
{
namespace amr
{
DeveloperModeSubscriber::DeveloperModeSubscriber(
  std::shared_ptr<AMRManager>& amr_manager,
  std::shared_ptr<ExceptionSubscriber>& exception_sub,
  std::shared_ptr<AMRStatusTransporter>& tansporter,
  const rclcpp::NodeOptions & options)
  : Node("developer_sub", options),
  amr_manager_(amr_manager),
  exception_sub_(exception_sub),
  tansporter_(tansporter)
{
  RCLCPP_INFO(logger_,"DeveloperModeSubscriber");
  init_subscription();
}

DeveloperModeSubscriber::~DeveloperModeSubscriber()
{
}

void DeveloperModeSubscriber::init_subscription()
{
  using namespace std::placeholders;

  sub_ = this->create_subscription<std_msgs::msg::Int16>(
         "test", 10, std::bind(&DeveloperModeSubscriber::sub_callback, this, _1));
  debug_error_sub_ = this->create_subscription<std_msgs::msg::Int16>(
         "debug_exception", 10, std::bind(&DeveloperModeSubscriber::debug_error_sub_callback, this, _1));
}

void DeveloperModeSubscriber::sub_callback(const std_msgs::msg::Int16::ConstSharedPtr msg)
{
  if (!enable_developer_mode_) {
     return;
  }
  RCLCPP_INFO(logger_,"subscribe callback");
  int16_t debug_msg = msg->data;
  send_debug_message(debug_msg);
}

void DeveloperModeSubscriber::send_debug_message(int msg)
{
  RCLCPP_INFO(logger_,"send_debug_message = %s", StringUtil::msg_to_string(msg).c_str());
  switch((DEBUG_MSG)msg) {
    case DEBUG_MSG::Init_AMR:
      amr_manager_->init_amr();
      break;
    case DEBUG_MSG::Release_AMR:
      amr_manager_->release_amr();
      break;
    case DEBUG_MSG::ME:
      amr_manager_->process_cmd(Message::ME, nullptr, 0);
      break;
    case DEBUG_MSG::AE:
      amr_manager_->process_cmd(Message::AE, nullptr, 0);
      break;
    case DEBUG_MSG::ME_Finish:
      amr_manager_->process_event(Message::ME_FINISH);
      break;
    case DEBUG_MSG::AE_Finish:
      amr_manager_->process_event(Message::AE_FINISH);
      break;
    case DEBUG_MSG::ME_Completed:
      amr_manager_->me_completed();
      break;
    case DEBUG_MSG::Pause:
      amr_manager_->process_event(Message::PAUSE);
      break;
    case DEBUG_MSG::Resume:
      amr_manager_->process_event(Message::RESUME);
      break;
    case DEBUG_MSG::Cancel:
      amr_manager_->process_event(Message::CANCEL);
      break;
    case DEBUG_MSG::P2PNav_Finish:
      amr_manager_->process_event(Message::P2PNAV_FINISH);
      break;
     case DEBUG_MSG::FollowPath_Finish:
      amr_manager_->process_event(Message::FOLLOW_PATH_FINISH);
      break;
     case DEBUG_MSG::Low_Power:
     {
      float battery_vol = 21;
      tansporter_->notify_battery_changed(battery_vol);
      break;
     }
     case DEBUG_MSG::Normal_Power:
     {
      float battery_vol = 24;
      tansporter_->notify_battery_changed(battery_vol);
      amr_manager_->process_event(Message::NORMAL_POWER);
      break;
     }
     case DEBUG_MSG::Return_Charging:
      amr_manager_->process_cmd(Message::RETURN_CHARGING, nullptr, 0);
      break;
     case DEBUG_MSG::Return_Charging_Finish:
      amr_manager_->process_event(Message::RETURN_CHARGING_FINISH);
      break;
     case DEBUG_MSG::AMR_Exception:
     {
      int error_code = 1;//this is test error code
      amr_manager_->process_event(Message::AMR_EXCEPTION, error_code);
      break;
     }
     case DEBUG_MSG::AMR_Normal:
      amr_manager_->process_event(Message::AMR_NORMAL);
      break;
     case DEBUG_MSG::Relocalization_Pass:
      amr_manager_->process_event(Message::RELOCALIZATION_PASS);
      break;
    default:
      RCLCPP_ERROR(logger_,"Not support debug message = %d", msg);
      break;
  }
}

void DeveloperModeSubscriber::set_developer_mode(bool val)
{
  enable_developer_mode_ = val;
  if (enable_developer_mode_) {
    RCLCPP_INFO(logger_,"Enable developer mode");
  } else {
    RCLCPP_INFO(logger_,"Disable developer mode");
  }
}

void DeveloperModeSubscriber::debug_error_sub_callback(const std_msgs::msg::Int16::ConstSharedPtr msg)
{
  if (!enable_developer_mode_) {
     return;
  }
  uint8_t debug_msg = msg->data;
  RCLCPP_INFO(logger_,"debug error subscribe callback, debug_msg=%d", debug_msg);
  exception_sub_->handle_debug_msg(debug_msg);
}
}  // namespace amr
}  // namespace qrb_ros
