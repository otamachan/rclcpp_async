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

#include "rclcpp_async/task.hpp"

using namespace rclcpp_async;  // NOLINT(build/namespaces)

// ============================================================
// Task<T>
// ============================================================

Task<int> returns_42() { co_return 42; }

TEST(TaskT, ReturnsValue)
{
  auto task = returns_42();
  // initial_suspend is suspend_always, so not done yet
  EXPECT_FALSE(task.handle.done());

  task.handle.resume();
  EXPECT_TRUE(task.handle.done());
  ASSERT_TRUE(task.handle.promise().value.has_value());
  EXPECT_EQ(*task.handle.promise().value, 42);
}

Task<std::string> returns_hello() { co_return "hello"; }

TEST(TaskT, ReturnsString)
{
  auto task = returns_hello();
  task.handle.resume();
  EXPECT_TRUE(task.handle.done());
  EXPECT_EQ(*task.handle.promise().value, "hello");
}

// ============================================================
// Task<void>
// ============================================================

Task<void> does_nothing() { co_return; }

TEST(TaskVoid, CompletesSuccessfully)
{
  auto task = does_nothing();
  EXPECT_FALSE(task.handle.done());

  task.handle.resume();
  EXPECT_TRUE(task.handle.done());
}

// ============================================================
// Coroutine chaining
// ============================================================

Task<int> inner_task() { co_return 10; }

Task<int> outer_task()
{
  int val = co_await inner_task();
  co_return val + 5;
}

TEST(TaskT, CoroutineChaining)
{
  auto task = outer_task();
  task.handle.resume();
  EXPECT_TRUE(task.handle.done());
  EXPECT_EQ(*task.handle.promise().value, 15);
}

Task<int> triple_chain()
{
  int a = co_await inner_task();  // 10
  int b = co_await inner_task();  // 10
  co_return a + b;
}

TEST(TaskT, MultipleCoAwait)
{
  auto task = triple_chain();
  task.handle.resume();
  EXPECT_TRUE(task.handle.done());
  EXPECT_EQ(*task.handle.promise().value, 20);
}

// ============================================================
// Cancel
// ============================================================

TEST(TaskT, CancelSetsToken)
{
  auto task = returns_42();
  EXPECT_FALSE(task.handle.promise().token.is_cancelled());
  task.cancel();
  EXPECT_TRUE(task.handle.promise().token.is_cancelled());
}

TEST(TaskVoid, CancelSetsToken)
{
  auto task = does_nothing();
  EXPECT_FALSE(task.handle.promise().token.is_cancelled());
  task.cancel();
  EXPECT_TRUE(task.handle.promise().token.is_cancelled());
}

// ============================================================
// Move semantics
// ============================================================

TEST(TaskT, MoveConstructor)
{
  auto task1 = returns_42();
  auto handle = task1.handle;
  auto task2 = std::move(task1);

  EXPECT_EQ(task1.handle, nullptr);
  EXPECT_EQ(task2.handle, handle);
}

// ============================================================
// await_transform with Cancellable concept
// ============================================================

struct MockCancellableAwaiter
{
  CancellationToken * token = nullptr;
  void set_token(CancellationToken * t) { token = t; }

  bool await_ready() { return true; }
  void await_suspend(std::coroutine_handle<>) {}
  int await_resume() { return 99; }
};

Task<int> uses_cancellable_awaiter()
{
  int val = co_await MockCancellableAwaiter{};
  co_return val;
}

TEST(TaskT, AwaitTransformSetsCancellationToken)
{
  auto task = uses_cancellable_awaiter();
  task.handle.resume();
  EXPECT_TRUE(task.handle.done());
  EXPECT_EQ(*task.handle.promise().value, 99);
}
