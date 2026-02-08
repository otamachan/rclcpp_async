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
#include <optional>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>

#include "rclcpp_async/cancellation_token.hpp"
#include "rclcpp_async/result.hpp"

namespace rclcpp_async
{

class CoContext;

template <typename MsgT>
class TopicStream
{
  CoContext & ctx_;
  std::queue<typename MsgT::SharedPtr> queue_;
  std::coroutine_handle<> waiter_;
  typename rclcpp::Subscription<MsgT>::SharedPtr sub_;
  bool closed_ = false;

  friend class CoContext;

public:
  explicit TopicStream(CoContext & ctx) : ctx_(ctx) {}

  struct NextAwaiter
  {
    using SharedPtr = typename MsgT::SharedPtr;

    TopicStream & stream;
    CancellationToken * token = nullptr;
    bool cancelled = false;

    void set_token(CancellationToken * t) { token = t; }

    bool await_ready()
    {
      if (token && token->is_cancelled()) {
        cancelled = true;
        return true;
      }
      return !stream.queue_.empty() || stream.closed_;
    }

    void await_suspend(std::coroutine_handle<> h);

    Result<std::optional<SharedPtr>> await_resume()
    {
      if (cancelled) {
        return Result<std::optional<SharedPtr>>::Cancelled();
      }
      if (stream.closed_ && stream.queue_.empty()) {
        return Result<std::optional<SharedPtr>>::Ok(std::nullopt);
      }
      auto msg = std::move(stream.queue_.front());
      stream.queue_.pop();
      return Result<std::optional<SharedPtr>>::Ok(std::move(msg));
    }
  };

  NextAwaiter next() { return NextAwaiter{*this, nullptr, false}; }

  void close();
};

}  // namespace rclcpp_async
