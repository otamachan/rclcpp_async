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
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "rclcpp_async/cancelled_exception.hpp"
#include "rclcpp_async/task.hpp"
#include "rclcpp_async/when_all.hpp"

namespace rclcpp_async
{

struct WhenAnyState : WhenAllState
{
  std::atomic<int> winner{-1};

  explicit WhenAnyState(int count) : WhenAllState(count) {}
};

struct RaceJoinTask
{
  struct promise_type
  {
    std::shared_ptr<WhenAnyState> state;
    std::exception_ptr exception;

    RaceJoinTask get_return_object()
    {
      return RaceJoinTask{std::coroutine_handle<promise_type>::from_promise(*this)};
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

    template <typename A>
    A && await_transform(A && awaiter)
    {
      return std::forward<A>(awaiter);
    }
  };

  std::coroutine_handle<promise_type> handle;

  ~RaceJoinTask()
  {
    if (handle) {
      handle.destroy();
    }
  }
  RaceJoinTask(RaceJoinTask && o) noexcept : handle(o.handle) { o.handle = nullptr; }
  RaceJoinTask & operator=(RaceJoinTask &&) = delete;
  RaceJoinTask(const RaceJoinTask &) = delete;

private:
  explicit RaceJoinTask(std::coroutine_handle<promise_type> h) : handle(h) {}
};

template <std::size_t I, typename T, typename Variant, std::size_t N>
RaceJoinTask race_join_coro(
  Task<T> task, std::shared_ptr<WhenAnyState> state, std::shared_ptr<Variant> result,
  std::shared_ptr<std::array<std::stop_source *, N>> sources)
{
  int expected = -1;
  try {
    if constexpr (std::is_void_v<T>) {
      co_await task;
      if (state->winner.compare_exchange_strong(expected, static_cast<int>(I))) {
        result->template emplace<I>(std::monostate{});
        for (std::size_t j = 0; j < N; ++j) {
          if (j != I) {
            (*sources)[j]->request_stop();
          }
        }
      }
    } else {
      auto val = co_await task;
      if (state->winner.compare_exchange_strong(expected, static_cast<int>(I))) {
        result->template emplace<I>(std::move(val));
        for (std::size_t j = 0; j < N; ++j) {
          if (j != I) {
            (*sources)[j]->request_stop();
          }
        }
      }
    }
  } catch (const CancelledException &) {
    if (state->winner.compare_exchange_strong(expected, static_cast<int>(I))) {
      throw;
    }
  } catch (...) {
    if (state->winner.compare_exchange_strong(expected, static_cast<int>(I))) {
      for (std::size_t j = 0; j < N; ++j) {
        if (j != I) {
          (*sources)[j]->request_stop();
        }
      }
      throw;
    }
  }
}

template <typename... Ts>
struct WhenAnyAwaiter
{
  static constexpr std::size_t N = sizeof...(Ts);
  using ResultVariant = std::variant<when_all_value_t<Ts>...>;

  std::shared_ptr<WhenAnyState> state;
  std::shared_ptr<ResultVariant> result;
  std::vector<RaceJoinTask> joins;
  std::array<std::stop_source *, N> child_sources;
  std::stop_token token;
  std::optional<StopCb> cancel_cb_;

  void set_token(std::stop_token t) { token = std::move(t); }

  bool await_ready() { return false; }

  bool await_suspend(std::coroutine_handle<> h)
  {
    state->continuation = h;

    if (token.stop_possible()) {
      cancel_cb_.emplace(token, [this]() {
        for (auto * ss : child_sources) {
          ss->request_stop();
        }
      });
    }

    for (auto & jt : joins) {
      jt.handle.resume();
    }

    return state->remaining.fetch_sub(1) != 1;
  }

  ResultVariant await_resume()
  {
    cancel_cb_.reset();
    int w = state->winner.load();
    if (w >= 0 && joins[w].handle.promise().exception) {
      std::rethrow_exception(joins[w].handle.promise().exception);
    }
    if (w < 0) {
      for (auto & jt : joins) {
        if (jt.handle.promise().exception) {
          std::rethrow_exception(jt.handle.promise().exception);
        }
      }
      throw CancelledException{};
    }
    return std::move(*result);
  }
};

template <typename... Ts>
WhenAnyAwaiter<Ts...> when_any(Task<Ts>... tasks)
{
  constexpr std::size_t N = sizeof...(Ts);

  std::array<std::stop_source *, N> child_sources = {&tasks.handle.promise().stop_source...};

  auto state = std::make_shared<WhenAnyState>(static_cast<int>(N + 1));
  auto result = std::make_shared<std::variant<when_all_value_t<Ts>...>>();
  auto sources = std::make_shared<std::array<std::stop_source *, N>>(child_sources);

  std::vector<RaceJoinTask> joins;
  joins.reserve(N);

  [&]<std::size_t... Is>(std::index_sequence<Is...>) {
    (joins.push_back(race_join_coro<Is>(std::move(tasks), state, result, sources)), ...);
  }(std::index_sequence_for<Ts...>{});

  for (auto & jt : joins) {
    jt.handle.promise().state = state;
  }

  return WhenAnyAwaiter<Ts...>{
    std::move(state), std::move(result), std::move(joins), child_sources, {}, {}};
}

}  // namespace rclcpp_async
