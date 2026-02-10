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
#include <tuple>
#include <variant>

#include "rclcpp_async/when_all.hpp"

using namespace rclcpp_async;  // NOLINT(build/namespaces)

using StopCb = std::stop_callback<std::function<void()>>;

Task<int> returns_int(int v) { co_return v; }

Task<std::string> returns_string(std::string s) { co_return s; }

Task<void> does_nothing() { co_return; }

Task<int> adds_two_tasks()
{
  int a = co_await returns_int(10);
  int b = co_await returns_int(20);
  co_return a + b;
}

Task<std::tuple<int, int>> two_int_tasks()
{
  co_return co_await when_all(returns_int(10), returns_int(20));
}

TEST(WhenAll, TwoIntTasks)
{
  auto task = two_int_tasks();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  auto [a, b] = *task.handle.promise().value;
  EXPECT_EQ(a, 10);
  EXPECT_EQ(b, 20);
}

Task<std::tuple<int, std::monostate>> mixed_void_and_value()
{
  co_return co_await when_all(returns_int(42), does_nothing());
}

TEST(WhenAll, MixedVoidAndValue)
{
  auto task = mixed_void_and_value();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  auto [a, b] = *task.handle.promise().value;
  EXPECT_EQ(a, 42);
  EXPECT_EQ(b, std::monostate{});
}

Task<std::tuple<int, std::string, int>> three_tasks()
{
  co_return co_await when_all(returns_int(1), returns_string("hello"), returns_int(3));
}

TEST(WhenAll, ThreeTasks)
{
  auto task = three_tasks();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  auto [a, b, c] = *task.handle.promise().value;
  EXPECT_EQ(a, 1);
  EXPECT_EQ(b, "hello");
  EXPECT_EQ(c, 3);
}

TEST(WhenAll, SynchronousCompletion)
{
  // All tasks complete without any async suspension
  auto task = two_int_tasks();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  auto [a, b] = *task.handle.promise().value;
  EXPECT_EQ(a, 10);
  EXPECT_EQ(b, 20);
}

Task<std::tuple<int, int>> cancel_propagation_coro(bool * child1_cancelled, bool * child2_cancelled)
{
  auto child1 = [&]() -> Task<int> {
    // Check token via cancellable awaiter pattern
    co_return 1;
  };
  auto child2 = [&]() -> Task<int> { co_return 2; };

  auto t1 = child1();
  auto t2 = child2();
  auto tok1 = t1.handle.promise().stop_source.get_token();
  auto tok2 = t2.handle.promise().stop_source.get_token();
  StopCb cb1(tok1, [child1_cancelled]() { *child1_cancelled = true; });
  StopCb cb2(tok2, [child2_cancelled]() { *child2_cancelled = true; });

  co_return co_await when_all(std::move(t1), std::move(t2));
}

TEST(WhenAll, CancelPropagation)
{
  bool child1_cancelled = false;
  bool child2_cancelled = false;
  auto task = cancel_propagation_coro(&child1_cancelled, &child2_cancelled);

  // Start but don't complete — cancel the parent
  task.cancel();

  // The parent token is now cancelled. Resume to enter when_all's await_suspend.
  task.handle.resume();

  EXPECT_TRUE(child1_cancelled);
  EXPECT_TRUE(child2_cancelled);
}

TEST(WhenAll, AlreadyCancelled)
{
  bool child1_cancelled = false;
  bool child2_cancelled = false;
  auto task = cancel_propagation_coro(&child1_cancelled, &child2_cancelled);

  // Cancel before resuming
  task.cancel();
  task.handle.resume();

  EXPECT_TRUE(child1_cancelled);
  EXPECT_TRUE(child2_cancelled);
}

Task<std::tuple<int>> single_task() { co_return co_await when_all(returns_int(99)); }

TEST(WhenAll, SingleTask)
{
  auto task = single_task();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  auto [a] = *task.handle.promise().value;
  EXPECT_EQ(a, 99);
}

Task<std::tuple<int, int>> chained_inner_tasks()
{
  co_return co_await when_all(adds_two_tasks(), returns_int(5));
}

TEST(WhenAll, ChainedInnerTasks)
{
  auto task = chained_inner_tasks();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  auto [a, b] = *task.handle.promise().value;
  EXPECT_EQ(a, 30);
  EXPECT_EQ(b, 5);
}
