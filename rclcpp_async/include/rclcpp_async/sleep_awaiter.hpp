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

#include <chrono>
#include <coroutine>
#include <functional>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <stop_token>
#include <utility>

#include "rclcpp_async/cancelled_exception.hpp"

namespace rclcpp_async
{

class CoContext;

using StopCb = std::stop_callback<std::function<void()>>;

struct SleepAwaiter
{
  CoContext & ctx;
  std::chrono::nanoseconds duration;
  rclcpp::TimerBase::SharedPtr timer;
  std::stop_token token;
  std::shared_ptr<StopCb> cancel_cb_;
  bool cancelled = false;
  bool done = false;

  void set_token(std::stop_token t) { token = std::move(t); }

  bool await_ready()
  {
    if (token.stop_requested()) {
      cancelled = true;
      return true;
    }
    if (duration <= std::chrono::nanoseconds{0}) {
      return true;
    }
    return false;
  }

  void await_suspend(std::coroutine_handle<> h);

  void await_resume()
  {
    cancel_cb_.reset();
    timer.reset();
    if (cancelled) {
      throw CancelledException{};
    }
  }
};

}  // namespace rclcpp_async
