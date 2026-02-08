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

struct WaitForServiceAwaiter
{
  CoContext & ctx;
  rclcpp::ClientBase::SharedPtr client;
  std::chrono::nanoseconds timeout;
  rclcpp::TimerBase::SharedPtr poll_timer;
  rclcpp::TimerBase::SharedPtr deadline_timer;
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
    if (client->service_is_ready()) {
      result = Result<void>::Ok();
      return true;
    }
    return false;
  }

  void await_suspend(std::coroutine_handle<> h);

  Result<void> await_resume()
  {
    poll_timer.reset();
    deadline_timer.reset();
    return std::move(result);
  }
};

}  // namespace rclcpp_async
