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
#include <functional>
#include <memory>
#include <optional>
#include <queue>
#include <stop_token>
#include <utility>

#include "rclcpp_async/cancelled_exception.hpp"

namespace rclcpp_async
{

class CoContext;

using StopCb = std::stop_callback<std::function<void()>>;

class Event
{
  CoContext & ctx_;
  bool set_ = false;

  struct Waiter
  {
    std::coroutine_handle<> handle;
    std::shared_ptr<bool> active;
  };
  std::queue<Waiter> waiters_;

public:
  explicit Event(CoContext & ctx) : ctx_(ctx) {}

  bool is_set() const { return set_; }

  void set();

  void clear() { set_ = false; }

  struct WaitAwaiter
  {
    Event & event;
    std::stop_token token;
    std::optional<StopCb> cancel_cb_;
    std::shared_ptr<bool> active;

    void set_token(std::stop_token t) { token = std::move(t); }

    bool await_ready()
    {
      if (token.stop_requested()) {
        return true;
      }
      return event.set_;
    }

    void await_suspend(std::coroutine_handle<> h);

    void await_resume()
    {
      cancel_cb_.reset();
      if (token.stop_requested()) {
        throw CancelledException{};
      }
    }
  };

  WaitAwaiter wait() { return WaitAwaiter{*this, {}, {}, nullptr}; }
};

}  // namespace rclcpp_async
