// Copyright 2025 Tamaki Nishino
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <coroutine>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace rclcpp_async
{

class CoContext;

class TimerStream
{
  rclcpp::TimerBase::SharedPtr timer_;
  std::coroutine_handle<> waiter_;
  int pending_ = 0;

  friend class CoContext;

public:
  TimerStream() = default;

  struct NextAwaiter
  {
    TimerStream & stream;

    bool await_ready()
    {
      if (stream.pending_ > 0) {
        stream.pending_--;
        return true;
      }
      return false;
    }

    void await_suspend(std::coroutine_handle<> h) { stream.waiter_ = h; }
    void await_resume() {}
  };

  NextAwaiter next() { return NextAwaiter{*this}; }

  void cancel()
  {
    if (timer_) {
      timer_->cancel();
    }
  }
};

}  // namespace rclcpp_async
