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
#include <mutex>
#include <optional>
#include <queue>
#include <utility>

#include "rclcpp_async/cancellation_token.hpp"

namespace rclcpp_async
{

class CoContext;

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
    CancellationToken * token = nullptr;
    bool cancelled = false;

    void set_token(CancellationToken * t) { token = t; }

    bool await_ready()
    {
      if (token && token->is_cancelled()) {
        cancelled = true;
        return true;
      }
      std::lock_guard lock(ch.mutex_);
      return !ch.queue_.empty() || ch.closed_;
    }

    bool await_suspend(std::coroutine_handle<> h);

    std::optional<T> await_resume()
    {
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

  NextAwaiter next() { return NextAwaiter{*this, nullptr, false}; }
};

}  // namespace rclcpp_async
