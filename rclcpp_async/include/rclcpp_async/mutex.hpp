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
#include "rclcpp_async/executor.hpp"

namespace rclcpp_async
{

class Mutex
{
  Executor & ctx_;
  bool locked_ = false;

  std::queue<std::weak_ptr<Waiting>> waiters_;

public:
  explicit Mutex(Executor & ctx) : ctx_(ctx) {}

  bool is_locked() const { return locked_; }

  struct LockAwaiter
  {
    Mutex & mutex;
    std::stop_token token;
    std::shared_ptr<StopCb> cancel_cb_;
    std::shared_ptr<Waiting> waiting_;

    void set_token(std::stop_token t) { token = std::move(t); }

    bool await_ready()
    {
      if (token.stop_requested()) {
        return true;
      }
      if (!mutex.locked_) {
        mutex.locked_ = true;
        return true;
      }
      return false;
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

  LockAwaiter lock() { return LockAwaiter{*this, {}, {}, nullptr}; }

  void unlock();
};

inline void Mutex::LockAwaiter::await_suspend(std::coroutine_handle<> h)
{
  waiting_ = std::make_shared<Waiting>(h);
  mutex.waiters_.push(waiting_);
  register_cancel(cancel_cb_, token, mutex.ctx_, waiting_, []() {});
}

inline void Mutex::unlock()
{
  while (!waiters_.empty()) {
    auto w = std::move(waiters_.front());
    waiters_.pop();
    if (resume_now(ctx_, w)) {
      return;
    }
  }
  locked_ = false;
}

}  // namespace rclcpp_async
