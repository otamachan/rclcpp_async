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

#include <gtest/gtest.h>

#include <functional>
#include <stop_token>
#include <string>
#include <variant>

#include "rclcpp_async/when_any.hpp"

using namespace rclcpp_async;  // NOLINT(build/namespaces)

using StopCb = std::stop_callback<std::function<void()>>;

struct IntAwaiter
{
  int value;
  bool await_ready() { return true; }
  void await_suspend(std::coroutine_handle<>) {}
  int await_resume() { return value; }
};

Task<int> returns_int(int v) { co_return v; }

Task<std::string> returns_string(std::string s) { co_return s; }

Task<void> does_nothing() { co_return; }

Task<int> adds_two_tasks()
{
  int a = co_await returns_int(10);
  int b = co_await returns_int(20);
  co_return a + b;
}

Task<std::variant<int, int>> two_int_race()
{
  co_return co_await when_any(returns_int(10), returns_int(20));
}

TEST(WhenAny, TwoIntTasks)
{
  auto task = two_int_race();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  auto result = *task.handle.promise().value;
  EXPECT_EQ(result.index(), 0u);
  EXPECT_EQ(std::get<0>(result), 10);
}

Task<std::variant<int, std::string>> mixed_types()
{
  co_return co_await when_any(returns_int(42), returns_string("hello"));
}

TEST(WhenAny, MixedTypes)
{
  auto task = mixed_types();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  auto result = *task.handle.promise().value;
  EXPECT_EQ(result.index(), 0u);
  EXPECT_EQ(std::get<0>(result), 42);
}

Task<std::variant<std::monostate, int>> void_and_value()
{
  co_return co_await when_any(does_nothing(), returns_int(5));
}

TEST(WhenAny, VoidAndValue)
{
  auto task = void_and_value();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  auto result = *task.handle.promise().value;
  EXPECT_EQ(result.index(), 0u);
}

Task<std::variant<int>> single_task() { co_return co_await when_any(returns_int(99)); }

TEST(WhenAny, SingleTask)
{
  auto task = single_task();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  auto result = *task.handle.promise().value;
  EXPECT_EQ(result.index(), 0u);
  EXPECT_EQ(std::get<0>(result), 99);
}

Task<std::variant<int, int>> cancel_propagation_coro(
  bool * child1_cancelled, bool * child2_cancelled)
{
  auto child1 = [&]() -> Task<int> { co_return 1; };
  auto child2 = [&]() -> Task<int> { co_return 2; };

  auto t1 = child1();
  auto t2 = child2();
  auto tok1 = t1.handle.promise().stop_source.get_token();
  auto tok2 = t2.handle.promise().stop_source.get_token();
  StopCb cb1(tok1, [child1_cancelled]() { *child1_cancelled = true; });
  StopCb cb2(tok2, [child2_cancelled]() { *child2_cancelled = true; });

  co_return co_await when_any(std::move(t1), std::move(t2));
}

TEST(WhenAny, CancelPropagation)
{
  bool child1_cancelled = false;
  bool child2_cancelled = false;
  auto task = cancel_propagation_coro(&child1_cancelled, &child2_cancelled);

  task.cancel();
  task.handle.resume();

  EXPECT_TRUE(child1_cancelled);
  EXPECT_TRUE(child2_cancelled);
}

Task<std::variant<int, int>> chained_inner()
{
  co_return co_await when_any(adds_two_tasks(), returns_int(5));
}

TEST(WhenAny, ChainedInnerTasks)
{
  auto task = chained_inner();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  auto result = *task.handle.promise().value;
  // adds_two_tasks completes first (synchronous), result is 30
  EXPECT_EQ(result.index(), 0u);
  EXPECT_EQ(std::get<0>(result), 30);
}

Task<std::variant<int, int>> two_awaitables_race()
{
  co_return co_await when_any(IntAwaiter{10}, IntAwaiter{20});
}

TEST(WhenAny, AwaitablesOnly)
{
  auto task = two_awaitables_race();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  auto result = *task.handle.promise().value;
  EXPECT_EQ(result.index(), 0u);
  EXPECT_EQ(std::get<0>(result), 10);
}

Task<std::variant<int, int>> mixed_task_and_awaitable_race()
{
  co_return co_await when_any(returns_int(1), IntAwaiter{2});
}

TEST(WhenAny, MixedTaskAndAwaitable)
{
  auto task = mixed_task_and_awaitable_race();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  auto result = *task.handle.promise().value;
  EXPECT_EQ(result.index(), 0u);
  EXPECT_EQ(std::get<0>(result), 1);
}
