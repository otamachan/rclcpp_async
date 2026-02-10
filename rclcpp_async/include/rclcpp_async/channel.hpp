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
#include <mutex>
#include <optional>
#include <queue>
#include <stop_token>
#include <utility>

#include "rclcpp_async/cancelled_exception.hpp"

namespace rclcpp_async
{

class CoContext;

using StopCb = std::stop_callback<std::function<void()>>;

template <typename T>
class Channel
{
  CoContext & ctx_;
  std::mutex mutex_;
  std::queue<T> queue_;
  std::coroutine_handle<> waiter_;
  bool closed_ = false;

public:
  explicit Channel(CoContext & ctx) : ctx_(ctx) {}

  void send(T value);
  void close();

  struct NextAwaiter
  {
    Channel & ch;
    std::stop_token token;
    std::unique_ptr<StopCb> cancel_cb_;
    bool cancelled = false;

    void set_token(std::stop_token t) { token = std::move(t); }

    bool await_ready()
    {
      if (token.stop_requested()) {
        cancelled = true;
        return true;
      }
      std::lock_guard lock(ch.mutex_);
      return !ch.queue_.empty() || ch.closed_;
    }

    bool await_suspend(std::coroutine_handle<> h);

    std::optional<T> await_resume()
    {
      cancel_cb_.reset();
      if (cancelled) {
        throw CancelledException{};
      }
      std::lock_guard lock(ch.mutex_);
      if (ch.queue_.empty()) {
        return std::nullopt;
      }
      auto val = std::move(ch.queue_.front());
      ch.queue_.pop();
      return std::move(val);
    }
  };

  NextAwaiter next() { return NextAwaiter{*this, {}, {}, false}; }
};

}  // namespace rclcpp_async
