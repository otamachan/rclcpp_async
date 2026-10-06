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
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <stop_token>
#include <utility>

#include "rclcpp_async/cancelled_exception.hpp"
#include "rclcpp_async/executor.hpp"

namespace rclcpp_async
{

template <typename T>
class Channel
{
  Executor & ctx_;
  std::mutex mutex_;
  std::queue<T> queue_;
  std::coroutine_handle<> waiter_;
  std::weak_ptr<void> waiter_alive_;
  size_t max_depth_;
  bool closed_ = false;

public:
  explicit Channel(Executor & ctx, size_t max_depth = kDefaultStreamDepth)
  : ctx_(ctx), max_depth_(max_depth)
  {
  }

  void send(T value);
  void close();

  struct NextAwaiter
  {
    Channel & ch;
    std::stop_token token;
    std::shared_ptr<StopCb> cancel_cb_;
    bool cancelled = false;
    std::shared_ptr<void> alive_;

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

  NextAwaiter next() { return NextAwaiter{*this, {}, {}, false, {}}; }
};

template <typename T>
bool Channel<T>::NextAwaiter::await_suspend(std::coroutine_handle<> h)
{
  {
    std::lock_guard lock(ch.mutex_);
    if (!ch.queue_.empty() || ch.closed_) {
      return false;  // don't suspend, data already available
    }
    alive_ = std::make_shared<char>();
    ch.waiter_ = h;
    ch.waiter_alive_ = alive_;
  }  // release ch.mutex_ before stop_callback to avoid deadlock
  cancel_cb_ = std::make_shared<StopCb>(token, [this, h, &cb = cancel_cb_]() {
    std::coroutine_handle<> w;
    {
      std::lock_guard lock(ch.mutex_);
      if (ch.waiter_ != h) {
        return;
      }
      w = ch.waiter_;
      ch.waiter_ = nullptr;
    }
    if (w) {
      cancelled = true;
      ch.ctx_.post([w, weak = std::weak_ptr(cb)]() {
        if (weak.lock()) {
          w.resume();
        }
      });
    }
  });
  return true;
}

template <typename T>
void Channel<T>::send(T value)
{
  std::coroutine_handle<> w;
  std::weak_ptr<void> alive;
  {
    std::lock_guard lock(mutex_);
    if (closed_) {
      return;
    }
    queue_.push(std::move(value));
    while (queue_.size() > max_depth_) {
      queue_.pop();
    }
    w = waiter_;
    waiter_ = nullptr;
    alive = std::move(waiter_alive_);
  }
  if (w) {
    post_resume(ctx_, w, std::move(alive));
  }
}

template <typename T>
void Channel<T>::close()
{
  std::coroutine_handle<> w;
  std::weak_ptr<void> alive;
  {
    std::lock_guard lock(mutex_);
    closed_ = true;
    w = waiter_;
    waiter_ = nullptr;
    alive = std::move(waiter_alive_);
  }
  if (w) {
    post_resume(ctx_, w, std::move(alive));
  }
}

}  // namespace rclcpp_async
