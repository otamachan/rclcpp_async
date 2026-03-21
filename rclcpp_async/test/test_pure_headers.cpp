// Copyright 2026 Tamaki Nishino
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

// This test verifies that pure C++ headers build without rclcpp.
// It is compiled with only the project include directory —
// no rclcpp, rclcpp_action, or tf2_ros on the include path.

#include <gtest/gtest.h>

#include <rclcpp_async/cancelled_exception.hpp>
#include <rclcpp_async/channel.hpp>
#include <rclcpp_async/event.hpp>
#include <rclcpp_async/executor.hpp>
#include <rclcpp_async/mutex.hpp>
#include <rclcpp_async/result.hpp>
#include <rclcpp_async/shield.hpp>
#include <rclcpp_async/task.hpp>
#include <rclcpp_async/when_all.hpp>
#include <rclcpp_async/when_any.hpp>

using namespace rclcpp_async;  // NOLINT(build/namespaces)

struct SimpleExecutor : Executor
{
  void post(std::function<void()> fn) override { fn(); }
};

TEST(PureHeaders, TaskReturnsValue)
{
  auto coro = []() -> Task<int> { co_return 42; };
  auto task = coro();
  task.started_ = true;
  task.handle.resume();
  ASSERT_TRUE(task.handle.done());
  EXPECT_EQ(task.result(), 42);
}

TEST(PureHeaders, EventSetBeforeWait)
{
  SimpleExecutor exec;
  Event event(exec);
  event.set();

  bool reached = false;
  auto coro = [&]() -> Task<void> {
    co_await event.wait();
    reached = true;
  };
  auto task = coro();
  task.started_ = true;
  task.handle.resume();

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(reached);
}

TEST(PureHeaders, EventWaitThenSet)
{
  SimpleExecutor exec;
  Event event(exec);

  bool reached = false;
  auto coro = [&]() -> Task<void> {
    co_await event.wait();
    reached = true;
  };
  auto task = coro();
  task.started_ = true;
  task.handle.resume();

  EXPECT_FALSE(reached);

  event.set();

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(reached);
}

TEST(PureHeaders, MutexLockUnlock)
{
  SimpleExecutor exec;
  Mutex mutex(exec);

  bool reached = false;
  auto coro = [&]() -> Task<void> {
    co_await mutex.lock();
    reached = true;
    mutex.unlock();
  };
  auto task = coro();
  task.started_ = true;
  task.handle.resume();

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(reached);
  EXPECT_FALSE(mutex.is_locked());
}

TEST(PureHeaders, ChannelSendReceive)
{
  SimpleExecutor exec;
  Channel<int> ch(exec);

  ch.send(42);

  std::optional<int> received;
  auto coro = [&]() -> Task<void> { received = co_await ch.next(); };
  auto task = coro();
  task.started_ = true;
  task.handle.resume();

  ASSERT_TRUE(task.handle.done());
  ASSERT_TRUE(received.has_value());
  EXPECT_EQ(*received, 42);
}

TEST(PureHeaders, ResultOkAndTimeout)
{
  auto ok = Result<int>::Ok(42);
  EXPECT_TRUE(ok.ok());
  EXPECT_EQ(*ok.value, 42);

  auto timeout = Result<int>::Timeout();
  EXPECT_TRUE(timeout.timeout());
}
