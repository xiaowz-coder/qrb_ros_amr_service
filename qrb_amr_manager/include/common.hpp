/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_AMR_MANAGER__COMMON_HPP_
#define QRB_AMR_MANAGER__COMMON_HPP_

#include <mutex>
#include <string>
#include <functional>
#include <vector>

using namespace std;

namespace qrb {
namespace amr_manager {

typedef struct {
  double x;
  double y;
  double z;
} twist_vel;

enum class Slam_Command {
  StartMapping = 1,
  StopMapping = 2,
  SaveMap = 3,
  LoadMap = 4,
  StartLocalization = 5,
  Relocalization = 6,
};

typedef std::function<void(void *buffer)> start_p2p_func_t;
typedef std::function<void(void *path)> start_follow_path_func_t;
typedef std::function<void(uint8_t goal, std::vector<uint32_t> &ids)>
    start_waypoint_follow_path_func_t;
typedef std::function<void(bool p2p, uint8_t sub_cmd)> sub_cmd_func_t;
typedef std::function<void(bool start)> start_charging_func_t;
typedef std::function<void(bool exception, int error_code)>
    notify_exception_func_t;
typedef std::function<void(int state)> send_amr_state_changed_func_t;
typedef std::function<void(void)> navigate_to_charging_func_t;
typedef std::function<void(uint8_t mode)> change_mode_func_t;
typedef std::function<void(uint8_t cmd, bool &result)> slam_command_func_t;
typedef std::function<void(twist_vel &velocity)>
    publish_twist_func_t;

class Command {
public:
  const static int ME = 1;
  const static int AE = 2;
  const static int P2PNAV = 3;
  const static int FOLLOW_PATH = 4;
  const static int CHARGING = 5;
  const static int WAYPOINT_FOLLOW_PATH = 6;
  const static int SUB_CMD = 7;
  const static int OTHER = 8;

  static std::string cmd_to_string(int cmd) {
    std::string message;
    switch (cmd) {
      case ME:
        message = "ME";
        break;
      case AE:
        message = "AE";
        break;
      case P2PNAV:
        message = "P2PNAV";
        break;
      case FOLLOW_PATH:
        message = "FOLLOW_PATH";
        break;
      case CHARGING:
        message = "CHARGING";
        break;
      case WAYPOINT_FOLLOW_PATH:
        message = "WAYPOINT_FOLLOW_PATH";
        break;
      case SUB_CMD:
        message = "SUB_CMD";
        break;
      case OTHER:
        message = "OTHER";
        break;
      default:
        message = "INVAILD";
        break;
    }
    return message;
  }
};

class SubCommand {
public:
  const static int CANCEL = 1;
  const static int PAUSE = 2;
  const static int RESUME = 3;

  static std::string to_string(int cmd) {
    std::string message;
    switch (cmd) {
    case CANCEL:
      message = "CANCEL";
      break;
    case PAUSE:
      message = "PAUSE";
      break;
    case RESUME:
      message = "RESUME";
      break;
    default:
      message = "INVAILD";
      break;
    }
    return message;
  }
};
} // namespace amr_manager
} // namespace qrb
#endif // QRB_AMR_MANAGER__COMMON_HPP_