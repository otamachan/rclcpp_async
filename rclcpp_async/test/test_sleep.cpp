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

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "rclcpp_async/rclcpp_async.hpp"

using namespace rclcpp_async;          // NOLINT(build/namespaces)
using namespace std::chrono_literals;  // NOLINT(build/namespaces)

class SleepTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_sleep_node");
    ctx_ = std::make_unique<CoContext>(node_);
  }

  void TearDown() override
  {
    ctx_.reset();
    node_.reset();
  }

  void spin_until_done(Task<void> & task, std::chrono::seconds timeout = 5s)
  {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!task.handle.done() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(1ms);
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<CoContext> ctx_;
};

TEST_F(SleepTest, SleepCompletesWithOk)
{
  Result<void> result;

  auto coro = [&]() -> Task<void> { result = co_await ctx_->sleep(50ms); };

  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(result.ok());
}

TEST_F(SleepTest, ZeroDurationIsImmediate)
{
  Result<void> result;

  auto coro = [&]() -> Task<void> { result = co_await ctx_->sleep(0ms); };

  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(result.ok());
}

TEST_F(SleepTest, CancelDuringSleep)
{
  Result<void> result;

  auto coro = [&]() -> Task<void> { result = co_await ctx_->sleep(10s); };

  auto task = ctx_->create_task(coro());

  // Let it start sleeping
  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(10ms);
  rclcpp::spin_some(node_);

  task.cancel();
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(result.cancelled());
}

TEST_F(SleepTest, AlreadyCancelledIsImmediate)
{
  Result<void> result;

  auto coro = [&]() -> Task<void> { result = co_await ctx_->sleep(10s); };

  auto task = coro();
  task.cancel();  // Cancel before starting
  auto running = ctx_->create_task(std::move(task));
  spin_until_done(running);

  ASSERT_TRUE(running.handle.done());
  EXPECT_TRUE(result.cancelled());
}

// co_await a create_task'd Task (like Python's await asyncio.create_task())
TEST_F(SleepTest, AwaitCreateTaskValue)
{
  int result = 0;

  auto background = [&]() -> Task<int> {
    co_await ctx_->sleep(50ms);
    co_return 42;
  };

  auto coro = [&]() -> Task<void> {
    auto bg = ctx_->create_task(background());
    result = co_await bg;
  };

  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_EQ(result, 42);
}

TEST_F(SleepTest, AwaitCreateTaskVoid)
{
  bool done = false;

  auto background = [&]() -> Task<void> {
    co_await ctx_->sleep(50ms);
    done = true;
  };

  auto coro = [&]() -> Task<void> {
    auto bg = ctx_->create_task(background());
    co_await bg;
  };

  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(done);
}

TEST_F(SleepTest, AwaitCreateTaskAlreadyDone)
{
  int result = 0;

  // Task that completes synchronously (no async suspension)
  auto background = [&]() -> Task<int> { co_return 99; };

  auto coro = [&]() -> Task<void> {
    auto bg = ctx_->create_task(background());
    // bg is already done by the time we co_await it
    result = co_await bg;
  };

  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_EQ(result, 99);
}
