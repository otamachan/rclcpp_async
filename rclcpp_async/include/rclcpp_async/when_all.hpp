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

#include <array>
#include <atomic>
#include <coroutine>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <stop_token>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "rclcpp_async/cancelled_exception.hpp"
#include "rclcpp_async/task.hpp"

namespace rclcpp_async
{

using StopCb = std::stop_callback<std::function<void()>>;

// Map Task<void> -> std::monostate, Task<T> -> T
template <typename T>
using when_all_value_t = std::conditional_t<std::is_void_v<T>, std::monostate, T>;

// Shared state for the when_all barrier
struct WhenAllState
{
  std::atomic<int> remaining;
  std::coroutine_handle<> continuation;

  explicit WhenAllState(int count) : remaining(count) {}
};

// Internal coroutine that wraps a single Task and stores its result
struct JoinTask
{
  struct promise_type
  {
    std::shared_ptr<WhenAllState> state;
    std::exception_ptr exception;

    JoinTask get_return_object()
    {
      return JoinTask{std::coroutine_handle<promise_type>::from_promise(*this)};
    }

    std::suspend_always initial_suspend() { return {}; }

    struct FinalAwaiter
    {
      bool await_ready() noexcept { return false; }
      std::coroutine_handle<> await_suspend(std::coroutine_handle<promise_type> h) noexcept
      {
        auto & st = *h.promise().state;
        if (st.remaining.fetch_sub(1) == 1) {
          return st.continuation;
        }
        return std::noop_coroutine();
      }
      void await_resume() noexcept {}
    };

    FinalAwaiter final_suspend() noexcept { return {}; }

    void return_void() {}
    void unhandled_exception() { exception = std::current_exception(); }

    // Pass-through await_transform — child Task already has its own token
    template <typename A>
    A && await_transform(A && awaiter)
    {
      return std::forward<A>(awaiter);
    }
  };

  std::coroutine_handle<promise_type> handle;

  ~JoinTask()
  {
    if (handle) {
      handle.destroy();
    }
  }
  JoinTask(JoinTask && o) noexcept : handle(o.handle) { o.handle = nullptr; }
  JoinTask & operator=(JoinTask &&) = delete;
  JoinTask(const JoinTask &) = delete;

private:
  explicit JoinTask(std::coroutine_handle<promise_type> h) : handle(h) {}
};

// Creates a JoinTask that awaits the given Task and stores the result
template <typename T>
JoinTask join_coro(Task<T> task, when_all_value_t<T> * slot)
{
  if constexpr (std::is_void_v<T>) {
    co_await task;
    *slot = std::monostate{};
  } else {
    *slot = co_await task;
  }
}

// Awaiter returned by when_all()
template <typename... Ts>
struct WhenAllAwaiter
{
  static constexpr std::size_t N = sizeof...(Ts);
  using ResultTuple = std::tuple<when_all_value_t<Ts>...>;

  std::shared_ptr<WhenAllState> state;
  std::shared_ptr<ResultTuple> results;
  std::vector<JoinTask> joins;
  std::array<std::stop_source *, N> child_sources;
  std::stop_token token;
  std::shared_ptr<StopCb> cancel_cb_;

  void set_token(std::stop_token t) { token = std::move(t); }

  bool await_ready() { return false; }

  bool await_suspend(std::coroutine_handle<> h)
  {
    state->continuation = h;

    // Register cancellation propagation
    if (token.stop_possible()) {
      cancel_cb_ = std::make_shared<StopCb>(token, [this]() {
        for (auto * ss : child_sources) {
          ss->request_stop();
        }
      });
    }

    // Resume all JoinTasks
    for (auto & jt : joins) {
      jt.handle.resume();
    }

    // Sentinel decrement: if all completed synchronously, don't suspend
    return state->remaining.fetch_sub(1) != 1;
  }

  ResultTuple await_resume()
  {
    cancel_cb_.reset();
    for (auto & jt : joins) {
      if (jt.handle.promise().exception) {
        std::rethrow_exception(jt.handle.promise().exception);
      }
    }
    return std::move(*results);
  }
};

// Factory function
template <typename... Ts>
WhenAllAwaiter<Ts...> when_all(Task<Ts>... tasks)
{
  constexpr std::size_t N = sizeof...(Ts);

  // Extract child stop_source pointers before moving tasks
  std::array<std::stop_source *, N> child_sources = {&tasks.handle.promise().stop_source...};

  auto state = std::make_shared<WhenAllState>(static_cast<int>(N + 1));
  auto results = std::make_shared<std::tuple<when_all_value_t<Ts>...>>();

  // Create JoinTasks via index_sequence
  std::vector<JoinTask> joins;
  joins.reserve(N);

  [&]<std::size_t... Is>(std::index_sequence<Is...>) {
    (joins.push_back(join_coro(std::move(tasks), &std::get<Is>(*results))), ...);
  }(std::index_sequence_for<Ts...>{});

  // Set shared state on each JoinTask's promise
  for (auto & jt : joins) {
    jt.handle.promise().state = state;
  }

  return WhenAllAwaiter<Ts...>{
    std::move(state), std::move(results), std::move(joins), child_sources, {}, {}};
}

template <typename... Awaitables>
auto when_all(Awaitables... awaitables)
  requires(!(is_task_v<Awaitables> && ...))
{
  return when_all(as_task(std::move(awaitables))...);
}

}  // namespace rclcpp_async
