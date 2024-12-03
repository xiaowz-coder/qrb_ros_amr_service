/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef COMMON_HPP_
#define COMMON_HPP_

namespace qrb
{
namespace amr_manager
{
class API
{
public:
  const static int INIT_AMR = 1;
  const static int RELEASE_AMR = 2;
  const static int ENABLE_DEVELOPER_MODE = 11;
  const static int ME_COMPLETED = 12;

  static std::string to_string(int cmd)
  {
    std::string message;
    switch (cmd) {
      case INIT_AMR:
        message = "INIT_AMR";
        break;
      case RELEASE_AMR:
        message = "RELEASE_AMR";
        break;
      case ENABLE_DEVELOPER_MODE:
        message = "ENABLE_DEVELOPER_MODE";
        break;
      case ME_COMPLETED:
        message = "ME_COMPLETED";
        break;
      default:
        message = "INVAILD API";
        break;
    }
    return message;
  }
};

}  // namespace amr_manager
}  // namsapce qrb
#endif // COMMON_HPP_