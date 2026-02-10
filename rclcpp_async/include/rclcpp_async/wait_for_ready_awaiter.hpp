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
#include <rclcpp_action/rclcpp_action.hpp>
#include <stop_token>
#include <utility>

#include "rclcpp_async/cancelled_exception.hpp"
#include "rclcpp_async/result.hpp"

namespace rclcpp_async
{

class CoContext;

using StopCb = std::stop_callback<std::function<void()>>;

struct ServiceReadyChecker
{
  rclcpp::ClientBase::SharedPtr client;
  bool is_ready() const { return client->service_is_ready(); }
};

template <typename ActionT>
struct ActionReadyChecker
{
  std::shared_ptr<rclcpp_action::Client<ActionT>> client;
  bool is_ready() const { return client->action_server_is_ready(); }
};

template <typename ReadyChecker>
struct WaitForReadyAwaiter
{
  CoContext & ctx;
  ReadyChecker checker;
  std::chrono::nanoseconds timeout;
  rclcpp::TimerBase::SharedPtr poll_timer;
  rclcpp::TimerBase::SharedPtr deadline_timer;
  std::stop_token token;
  std::optional<StopCb> cancel_cb_;
  Result<void> result;
  bool done = false;

  void set_token(std::stop_token t) { token = std::move(t); }

  bool cancelled = false;

  bool await_ready()
  {
    if (token.stop_requested()) {
      cancelled = true;
      return true;
    }
    if (checker.is_ready()) {
      result = Result<void>::Ok();
      return true;
    }
    return false;
  }

  void await_suspend(std::coroutine_handle<> h);

  Result<void> await_resume()
  {
    cancel_cb_.reset();
    poll_timer.reset();
    deadline_timer.reset();
    if (cancelled) {
      throw CancelledException{};
    }
    return std::move(result);
  }
};

// Type aliases for backward compatibility
using WaitForServiceAwaiter = WaitForReadyAwaiter<ServiceReadyChecker>;

template <typename ActionT>
using WaitForActionAwaiter = WaitForReadyAwaiter<ActionReadyChecker<ActionT>>;

}  // namespace rclcpp_async
