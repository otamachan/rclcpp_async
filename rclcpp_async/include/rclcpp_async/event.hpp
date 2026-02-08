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
#include <queue>

namespace rclcpp_async
{

class CoContext;

class Event
{
  CoContext & ctx_;
  bool set_ = false;
  std::queue<std::coroutine_handle<>> waiters_;

public:
  explicit Event(CoContext & ctx) : ctx_(ctx) {}

  bool is_set() const { return set_; }

  void set();

  void clear() { set_ = false; }

  struct WaitAwaiter
  {
    Event & event;

    bool await_ready() { return event.set_; }
    void await_suspend(std::coroutine_handle<> h) { event.waiters_.push(h); }
    void await_resume() {}
  };

  WaitAwaiter wait() { return WaitAwaiter{*this}; }
};

}  // namespace rclcpp_async
