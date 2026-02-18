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
#include <functional>
#include <memory>
#include <optional>
#include <stop_token>
#include <type_traits>
#include <utility>

#include "rclcpp_async/cancelled_exception.hpp"

namespace rclcpp_async
{

template <typename T>
concept Cancellable =
  requires(T t, std::stop_token token) { t.set_token(token); };  // NOLINT(readability/braces)

template <typename T = void>
struct Task
{
  struct promise_type
  {
    std::optional<T> value;
    std::exception_ptr exception;
    std::coroutine_handle<> continuation;
    std::stop_source stop_source;

    Task get_return_object()
    {
      return Task{std::coroutine_handle<promise_type>::from_promise(*this)};
    }

    std::suspend_always initial_suspend() { return {}; }

    struct FinalAwaiter
    {
      bool await_ready() noexcept { return false; }
      std::coroutine_handle<> await_suspend(std::coroutine_handle<promise_type> h) noexcept
      {
        if (h.promise().continuation) {
          return h.promise().continuation;
        }
        return std::noop_coroutine();
      }
      void await_resume() noexcept {}
    };

    FinalAwaiter final_suspend() noexcept { return {}; }

    template <Cancellable A>
    A && await_transform(A && awaiter)
    {
      awaiter.set_token(stop_source.get_token());
      return std::forward<A>(awaiter);
    }

    template <typename A>
      requires(!Cancellable<A>)
    A && await_transform(A && awaiter)
    {
      return std::forward<A>(awaiter);
    }

    void return_value(T v) { value = std::move(v); }
    void unhandled_exception() { exception = std::current_exception(); }
  };

  using StopCb = std::stop_callback<std::function<void()>>;

  std::coroutine_handle<promise_type> handle;
  bool started_ = false;
  std::stop_token parent_token_;
  std::unique_ptr<StopCb> parent_cancel_cb_;

  void set_token(std::stop_token t) { parent_token_ = std::move(t); }

  // co_await Task (coroutine chaining)
  bool await_ready() { return handle.done(); }
  std::coroutine_handle<> await_suspend(std::coroutine_handle<> h)
  {
    handle.promise().continuation = h;
    if (parent_token_.stop_possible()) {
      auto child = handle;
      parent_cancel_cb_ = std::unique_ptr<StopCb>(
        new StopCb(parent_token_, [child]() { child.promise().stop_source.request_stop(); }));
    }
    if (started_) {
      return std::noop_coroutine();
    }
    started_ = true;
    return handle;
  }
  T await_resume()
  {
    parent_cancel_cb_.reset();
    if (handle.promise().exception) {
      std::rethrow_exception(handle.promise().exception);
    }
    return std::move(*handle.promise().value);
  }

  T & result()
  {
    if (handle.promise().exception) {
      std::rethrow_exception(handle.promise().exception);
    }
    return *handle.promise().value;
  }

  void cancel() { handle.promise().stop_source.request_stop(); }

  explicit operator bool() const noexcept { return handle != nullptr; }
  bool done() const noexcept { return handle && handle.done(); }

  ~Task()
  {
    if (handle) {
      handle.promise().stop_source.request_stop();
      handle.destroy();
    }
  }
  Task(Task && o) noexcept
  : handle(o.handle), started_(o.started_), parent_cancel_cb_(std::move(o.parent_cancel_cb_))
  {
    o.handle = nullptr;
  }
  Task & operator=(Task && o) noexcept
  {
    if (this != &o) {
      if (handle) {
        handle.destroy();
      }
      handle = o.handle;
      started_ = o.started_;
      parent_cancel_cb_ = std::move(o.parent_cancel_cb_);
      o.handle = nullptr;
    }
    return *this;
  }
  Task(const Task &) = delete;
  Task() noexcept : handle(nullptr) {}

private:
  explicit Task(std::coroutine_handle<promise_type> h) : handle(h) {}
};

// void specialization
template <>
struct Task<void>
{
  struct promise_type
  {
    std::exception_ptr exception;
    std::coroutine_handle<> continuation;
    std::stop_source stop_source;

    Task get_return_object()
    {
      return Task{std::coroutine_handle<promise_type>::from_promise(*this)};
    }

    std::suspend_always initial_suspend() { return {}; }

    struct FinalAwaiter
    {
      bool await_ready() noexcept { return false; }
      std::coroutine_handle<> await_suspend(std::coroutine_handle<promise_type> h) noexcept
      {
        if (h.promise().continuation) {
          return h.promise().continuation;
        }
        return std::noop_coroutine();
      }
      void await_resume() noexcept {}
    };

    FinalAwaiter final_suspend() noexcept { return {}; }

    template <Cancellable A>
    A && await_transform(A && awaiter)
    {
      awaiter.set_token(stop_source.get_token());
      return std::forward<A>(awaiter);
    }

    template <typename A>
      requires(!Cancellable<A>)
    A && await_transform(A && awaiter)
    {
      return std::forward<A>(awaiter);
    }

    void return_void() {}
    void unhandled_exception() { exception = std::current_exception(); }
  };

  using StopCb = std::stop_callback<std::function<void()>>;

  std::coroutine_handle<promise_type> handle;
  bool started_ = false;
  std::stop_token parent_token_;
  std::unique_ptr<StopCb> parent_cancel_cb_;

  void set_token(std::stop_token t) { parent_token_ = std::move(t); }

  bool await_ready() { return handle.done(); }
  std::coroutine_handle<> await_suspend(std::coroutine_handle<> h)
  {
    handle.promise().continuation = h;
    if (parent_token_.stop_possible()) {
      auto child = handle;
      parent_cancel_cb_ = std::unique_ptr<StopCb>(
        new StopCb(parent_token_, [child]() { child.promise().stop_source.request_stop(); }));
    }
    if (started_) {
      return std::noop_coroutine();
    }
    started_ = true;
    return handle;
  }
  void await_resume()
  {
    parent_cancel_cb_.reset();
    if (handle.promise().exception) {
      std::rethrow_exception(handle.promise().exception);
    }
  }

  void cancel() { handle.promise().stop_source.request_stop(); }

  explicit operator bool() const noexcept { return handle != nullptr; }
  bool done() const noexcept { return handle && handle.done(); }

  ~Task()
  {
    if (handle) {
      handle.promise().stop_source.request_stop();
      handle.destroy();
    }
  }
  Task(Task && o) noexcept
  : handle(o.handle), started_(o.started_), parent_cancel_cb_(std::move(o.parent_cancel_cb_))
  {
    o.handle = nullptr;
  }
  Task & operator=(Task && o) noexcept
  {
    if (this != &o) {
      if (handle) {
        handle.destroy();
      }
      handle = o.handle;
      started_ = o.started_;
      parent_cancel_cb_ = std::move(o.parent_cancel_cb_);
      o.handle = nullptr;
    }
    return *this;
  }
  Task(const Task &) = delete;
  Task() noexcept : handle(nullptr) {}

private:
  explicit Task(std::coroutine_handle<promise_type> h) : handle(h) {}
};

// Fire-and-forget coroutine that starts immediately and self-destructs on
// completion. Used internally for service server callbacks where co_await is
// needed.
struct SpawnedTask
{
  struct promise_type
  {
    std::stop_source stop_source;

    SpawnedTask get_return_object() { return {}; }

    std::suspend_never initial_suspend() { return {}; }
    std::suspend_never final_suspend() noexcept { return {}; }

    template <Cancellable A>
    A && await_transform(A && awaiter)
    {
      awaiter.set_token(stop_source.get_token());
      return std::forward<A>(awaiter);
    }

    template <typename A>
      requires(!Cancellable<A>)
    A && await_transform(A && awaiter)
    {
      return std::forward<A>(awaiter);
    }

    void return_void() {}
    void unhandled_exception()
    {
      try {
        std::rethrow_exception(std::current_exception());
      } catch (const CancelledException &) {
        // Swallow cancellation in fire-and-forget tasks
      }
    }
  };
};

template <typename T>
inline constexpr bool is_task_v = false;
template <typename T>
inline constexpr bool is_task_v<Task<T>> = true;

template <typename A>
auto as_task(A a)
{
  if constexpr (is_task_v<A>) {
    return std::move(a);
  } else {
    using T = std::decay_t<decltype(a.await_resume())>;
    if constexpr (std::is_void_v<T>) {
      return [](A x) -> Task<void> { co_await std::move(x); }(std::move(a));
    } else {
      return [](A x) -> Task<T> { co_return co_await std::move(x); }(std::move(a));
    }
  }
}

}  // namespace rclcpp_async
