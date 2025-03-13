/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_AMR_MANAGER__MESSAGE_HPP_
#define QRB_AMR_MANAGER__MESSAGE_HPP_

#include <memory>
#include <string>
#include <vector>

namespace qrb
{
namespace amr_manager
{

class Message
{
public:
  const static int AE_FINISH = 1;
  const static int P2PNAV_FINISH = 2;
  const static int FOLLOW_PATH_FINISH = 3;
  const static int LOW_POWER = 4;
  const static int NORMAL_POWER = 5;
  const static int RETURN_CHARGING_FINISH = 6;
  const static int AMR_EXCEPTION = 7;
  const static int AMR_NORMAL = 8;
  const static int ME_FINISH = 9;
  const static int RETURN_CHARGING = 10;

  const static int ME = 21;
  const static int AE = 22;
  const static int P2PNAV = 23;
  const static int FOLLOW_PATH = 24;
  const static int WAYPOINT_FOLLOW_PATH = 25;

  const static int CANCEL = 31;
  const static int PAUSE = 32;
  const static int RESUME = 33;

  const static int INIT_AMR = 41;
  const static int RELEASE_AMR = 42;
  const static int ME_COMPLETED = 43;
  const static int RELOCALIZATION_PASS = 44;

  int type;
  void * param;
  size_t len;
  uint32_t goal_id;
  uint8_t error_code;
  std::vector<uint32_t> ids;

  static std::string msg_to_string(int msg)
  {
    std::string message;
    switch (msg) {
      case AE_FINISH:
        message = "AE_FINISH";
        break;
      case ME_FINISH:
        message = "ME_FINISH";
        break;
      case P2PNAV_FINISH:
        message = "P2PNAV_FINISH";
        break;
      case FOLLOW_PATH_FINISH:
        message = "FOLLOW_PATH_FINISH";
        break;
      case NORMAL_POWER:
        message = "NORMAL_POWER";
        break;
      case LOW_POWER:
        message = "LOW_POWER";
        break;
      case RETURN_CHARGING_FINISH:
        message = "RETURN_CHARGING_FINISH";
        break;
      case RETURN_CHARGING:
        message = "RETURN_CHARGING";
        break;
      case AMR_EXCEPTION:
        message = "AMR_EXCEPTION";
        break;
      case AMR_NORMAL:
        message = "AMR_NORMAL";
        break;
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
      case RELEASE_AMR:
        message = "RELEASE_AMR";
        break;
      case INIT_AMR:
        message = "INIT_AMR";
        break;
      case ME_COMPLETED:
        message = "ME_COMPLETED";
        break;
      case RESUME:
        message = "RESUME";
        break;
      case PAUSE:
        message = "PAUSE";
        break;
      case CANCEL:
        message = "CANCEL";
        break;
      case RELOCALIZATION_PASS:
        message = "RELOCALIZATION_PASS";
        break;
      default:
        message = "INVALID";
        break;
    }
    return message;
  }
};
}  // namespace amr_manager
}  // namespace qrb
#endif  // QRB_AMR_MANAGER__MESSAGE_HPP_