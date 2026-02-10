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

class PollWaitForTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_poll_wait_for_node");
    ctx_ = std::make_unique<CoContext>(node_);
    executor_.add_node(node_);
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
      executor_.spin_some();
      std::this_thread::sleep_for(1ms);
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<CoContext> ctx_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(PollWaitForTest, PollImmediatelyReady)
{
  bool reached = false;
  auto coro = [&]() -> Task<void> {
    co_await ctx_->poll([]() { return true; }, 100ms);
    reached = true;
  };
  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(reached);
}

TEST_F(PollWaitForTest, PollBecomesReady)
{
  int counter = 0;
  bool reached = false;
  auto coro = [&]() -> Task<void> {
    co_await ctx_->poll([&]() { return counter >= 3; }, 10ms);
    reached = true;
  };
  auto task = ctx_->create_task(coro());

  auto timer = node_->create_wall_timer(10ms, [&counter]() { counter++; });
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(reached);
  EXPECT_GE(counter, 3);
}

TEST_F(PollWaitForTest, WaitForCompletes)
{
  Result<void> res = Result<void>::Timeout();
  auto coro = [&]() -> Task<void> {
    auto inner = [this]() -> Task<void> { co_await ctx_->sleep(10ms); };
    res = co_await ctx_->wait_for(inner(), 5s);
  };
  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(res.ok());
}

TEST_F(PollWaitForTest, WaitForTimesOut)
{
  Result<void> res = Result<void>::Ok();
  auto coro = [&]() -> Task<void> {
    auto inner = [this]() -> Task<void> { co_await ctx_->sleep(5s); };
    res = co_await ctx_->wait_for(inner(), 50ms);
  };
  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(res.timeout());
}

TEST_F(PollWaitForTest, PollWithTimeout)
{
  int counter = 0;
  Result<void> res = Result<void>::Timeout();
  auto coro = [&]() -> Task<void> {
    auto timer = node_->create_wall_timer(10ms, [&counter]() { counter++; });
    res = co_await ctx_->wait_for(ctx_->poll([&]() { return counter >= 3; }, 10ms), 5s);
  };
  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(res.ok());
  EXPECT_GE(counter, 3);
}

TEST_F(PollWaitForTest, PollWithTimeoutTimesOut)
{
  Result<void> res = Result<void>::Ok();
  auto coro = [&]() -> Task<void> {
    res = co_await ctx_->wait_for(ctx_->poll([]() { return false; }, 10ms), 50ms);
  };
  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(res.timeout());
}
