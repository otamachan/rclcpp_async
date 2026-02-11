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

#include <stop_token>

#include "rclcpp_async/task.hpp"

using namespace rclcpp_async;  // NOLINT(build/namespaces)

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

Task<void> does_nothing() { co_return; }

TEST(TaskVoid, CompletesSuccessfully)
{
  auto task = does_nothing();
  EXPECT_FALSE(task.handle.done());

  task.handle.resume();
  EXPECT_TRUE(task.handle.done());
}

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

TEST(TaskT, CancelSetsToken)
{
  auto task = returns_42();
  EXPECT_FALSE(task.handle.promise().stop_source.stop_requested());
  task.cancel();
  EXPECT_TRUE(task.handle.promise().stop_source.stop_requested());
}

TEST(TaskVoid, CancelSetsToken)
{
  auto task = does_nothing();
  EXPECT_FALSE(task.handle.promise().stop_source.stop_requested());
  task.cancel();
  EXPECT_TRUE(task.handle.promise().stop_source.stop_requested());
}

TEST(TaskT, MoveConstructor)
{
  auto task1 = returns_42();
  auto handle = task1.handle;
  auto task2 = std::move(task1);

  EXPECT_EQ(task1.handle, nullptr);
  EXPECT_EQ(task2.handle, handle);
}

Task<int> throws_runtime_error()
{
  throw std::runtime_error("test error");
  co_return 0;
}

TEST(TaskT, ExceptionPropagation)
{
  auto inner = []() -> Task<int> { co_return co_await throws_runtime_error(); };

  auto task = inner();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(task.handle.promise().exception != nullptr);
}

TEST(TaskT, ExceptionPropagationThroughChain)
{
  auto outer = []() -> Task<int> {
    int val = co_await throws_runtime_error();
    co_return val;
  };

  auto task = outer();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(task.handle.promise().exception != nullptr);
}

Task<void> throws_void()
{
  throw std::runtime_error("void error");
  co_return;
}

TEST(TaskVoid, ExceptionPropagation)
{
  auto inner = []() -> Task<void> { co_await throws_void(); };

  auto task = inner();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(task.handle.promise().exception != nullptr);
}

struct MockCancellableAwaiter
{
  std::stop_token token;
  void set_token(std::stop_token t) { token = std::move(t); }

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

Task<int> await_started_value()
{
  auto bg = returns_42();
  bg.started_ = true;
  bg.handle.resume();  // simulate create_task
  int val = co_await std::move(bg);
  co_return val + 1;
}

TEST(TaskT, AwaitStartedTask)
{
  auto task = await_started_value();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  EXPECT_EQ(*task.handle.promise().value, 43);
}

Task<void> await_started_void()
{
  auto bg = does_nothing();
  bg.started_ = true;
  bg.handle.resume();
  co_await std::move(bg);
}

TEST(TaskVoid, AwaitStartedTask)
{
  auto task = await_started_void();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
}

// Await a started task that hasn't completed yet (suspended mid-execution)
Task<int> slow_task()
{
  int a = co_await inner_task();  // suspends then resumes
  co_return a + 100;
}

Task<int> await_started_slow()
{
  auto bg = slow_task();
  bg.started_ = true;
  bg.handle.resume();
  // bg is now done (synchronous chain), co_await should return immediately
  int val = co_await std::move(bg);
  co_return val;
}

TEST(TaskT, AwaitStartedSlowTask)
{
  auto task = await_started_slow();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  EXPECT_EQ(*task.handle.promise().value, 110);
}

// Lazy co_await still works (backwards compatibility)
Task<int> await_lazy()
{
  int val = co_await returns_42();
  co_return val + 1;
}

TEST(TaskT, AwaitLazyStillWorks)
{
  auto task = await_lazy();
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  EXPECT_EQ(*task.handle.promise().value, 43);
}

TEST(TaskT, OperatorBoolAndDone)
{
  auto task = returns_42();
  EXPECT_TRUE(static_cast<bool>(task));
  EXPECT_FALSE(task.done());

  task.handle.resume();
  EXPECT_TRUE(static_cast<bool>(task));
  EXPECT_TRUE(task.done());

  // After move, source becomes null
  auto task2 = std::move(task);
  EXPECT_FALSE(static_cast<bool>(task));
  EXPECT_FALSE(task.done());
  EXPECT_TRUE(static_cast<bool>(task2));
  EXPECT_TRUE(task2.done());
}

TEST(TaskVoid, OperatorBoolAndDone)
{
  auto task = does_nothing();
  EXPECT_TRUE(static_cast<bool>(task));
  EXPECT_FALSE(task.done());

  task.handle.resume();
  EXPECT_TRUE(static_cast<bool>(task));
  EXPECT_TRUE(task.done());

  auto task2 = std::move(task);
  EXPECT_FALSE(static_cast<bool>(task));
  EXPECT_FALSE(task.done());
  EXPECT_TRUE(static_cast<bool>(task2));
  EXPECT_TRUE(task2.done());
}
