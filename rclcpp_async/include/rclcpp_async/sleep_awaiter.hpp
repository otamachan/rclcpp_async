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
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "rclcpp_async/cancellation_token.hpp"
#include "rclcpp_async/result.hpp"

namespace rclcpp_async
{

class CoContext;

struct SleepAwaiter
{
  CoContext & ctx;
  std::chrono::nanoseconds duration;
  rclcpp::TimerBase::SharedPtr timer;
  CancellationToken * token = nullptr;
  Result<void> result;
  bool done = false;

  void set_token(CancellationToken * t) { token = t; }

  bool await_ready()
  {
    if (token && token->is_cancelled()) {
      result = Result<void>::Cancelled();
      return true;
    }
    if (duration <= std::chrono::nanoseconds{0}) {
      result = Result<void>::Ok();
      return true;
    }
    return false;
  }

  void await_suspend(std::coroutine_handle<> h);

  Result<void> await_resume()
  {
    timer.reset();
    return std::move(result);
  }
};

}  // namespace rclcpp_async
