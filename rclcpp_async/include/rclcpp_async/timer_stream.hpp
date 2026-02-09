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

#include "rclcpp_async/cancellation_token.hpp"

namespace rclcpp_async
{

class CoContext;

class TimerStream
{
  CoContext & ctx_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::coroutine_handle<> waiter_;
  int pending_ = 0;

  friend class CoContext;

public:
  explicit TimerStream(CoContext & ctx) : ctx_(ctx) {}

  struct NextAwaiter
  {
    TimerStream & stream;
    CancellationToken * token = nullptr;
    bool cancelled = false;

    void set_token(CancellationToken * t) { token = t; }

    bool await_ready()
    {
      if (token && token->is_cancelled()) {
        cancelled = true;
        return true;
      }
      if (stream.pending_ > 0) {
        stream.pending_--;
        return true;
      }
      return false;
    }

    void await_suspend(std::coroutine_handle<> h);

    void await_resume()
    {
      if (cancelled) {
        throw CancelledException{};
      }
    }
  };

  NextAwaiter next() { return NextAwaiter{*this, nullptr, false}; }

  void cancel()
  {
    if (timer_) {
      timer_->cancel();
    }
  }
};

}  // namespace rclcpp_async
