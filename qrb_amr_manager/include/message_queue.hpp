/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef MESSAGE_QUEUE_HPP_
#define MESSAGE_QUEUE_HPP_

#include "message.hpp"

#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>

namespace qrb {
namespace amr_manager {

class MessageQueue {

public:
  void push(const Message &msg) {
    std::unique_lock<std::mutex> lck(mtx_);
    if (msg.type == Message::AMR_EXCEPTION) {
      is_error_ = true;
      error_code_ = msg.error_code;
    } else if (msg.type == Message::LOW_POWER) {
      is_low_power_ = true;
    } else if (msg.type == Message::INIT_AMR) {
      is_init_ = true;
    } else if (msg.type == Message::RELEASE_AMR) {
      is_release_ = true;
    } else {
      queue_.push(msg);
    }
    cv_.notify_one();
  }

  void wait(Message &msg) {
    std::unique_lock<std::mutex> lck(mtx_);
    while (!queue_.size() && !is_error_ && !is_low_power_ && !is_init_ &&
           !is_release_)
      cv_.wait(lck);
    if (is_init_) {
      msg.type = Message::INIT_AMR;
      is_init_ = false;
    } else if (is_release_) {
      msg.type = Message::RELEASE_AMR;
      is_release_ = false;
    } else if (is_error_) {
      msg.type = Message::AMR_EXCEPTION;
      msg.error_code = error_code_;
      is_error_ = false;
    } else if (is_low_power_) {
      msg.type = Message::LOW_POWER;
      is_low_power_ = false;
    } else {
      msg = queue_.front();
      queue_.pop();
    }
  }

  size_t size() {
    std::unique_lock<std::mutex> lck(mtx_);
    return queue_.size();
  }

  void notify() {
    std::unique_lock<std::mutex> lck(mtx_);
    Message msg;
    msg.type = 0;
    queue_.push(msg);
    cv_.notify_one();
  }

private:
  std::queue<Message> queue_;
  std::mutex mtx_;
  std::condition_variable cv_;
  bool is_error_ = false;
  uint8_t error_code_ = 0;
  bool is_low_power_ = false;
  bool is_init_ = false;
  bool is_release_ = false;
};
} // namespace amr_manager
} // namespace qrb
#endif // MESSAGE_QUEUE_HPP_