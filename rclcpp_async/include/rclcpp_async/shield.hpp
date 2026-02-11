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
#include <exception>
#include <type_traits>
#include <utility>

#include "rclcpp_async/task.hpp"

namespace rclcpp_async
{

namespace detail
{

/// Awaiter that wraps a Task but does NOT implement set_token,
/// so it is not Cancellable. The parent's stop_token is not propagated
/// to the inner task, allowing it to run to completion even if the
/// parent is cancelled.
template <typename T>
struct ShieldAwaiter
{
  Task<T> task_;

  bool await_ready() { return task_.handle.done(); }

  std::coroutine_handle<> await_suspend(std::coroutine_handle<> cont)
  {
    task_.handle.promise().continuation = cont;
    if (task_.started_) {
      return std::noop_coroutine();
    }
    task_.started_ = true;
    return task_.handle;
  }

  T await_resume()
  {
    if (task_.handle.promise().exception) {
      std::rethrow_exception(task_.handle.promise().exception);
    }
    if constexpr (!std::is_void_v<T>) {
      return std::move(*task_.handle.promise().value);
    }
  }

  // No set_token — intentionally not Cancellable.
};

}  // namespace detail

/// Shield a task from cancellation. The inner task will run to completion
/// even if the parent coroutine's stop_token is triggered.
///
/// Usage:
///   auto result = co_await shield(do_critical_work());
template <typename T>
detail::ShieldAwaiter<T> shield(Task<T> task)
{
  return {std::move(task)};
}

}  // namespace rclcpp_async
