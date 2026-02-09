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

class EventTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_event_node");
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

TEST_F(EventTest, AlreadySet)
{
  Event event(*ctx_);
  event.set();

  bool reached = false;
  auto coro = [&]() -> Task<void> {
    co_await event.wait();
    reached = true;
  };
  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(reached);
}

TEST_F(EventTest, WaitThenSet)
{
  Event event(*ctx_);

  bool reached = false;
  auto coro = [&]() -> Task<void> {
    co_await event.wait();
    reached = true;
  };
  auto task = ctx_->create_task(coro());

  // Spin a bit — task should be suspended
  for (int i = 0; i < 10; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_FALSE(reached);

  // Set the event via a timer
  auto timer = node_->create_wall_timer(10ms, [&event]() { event.set(); });
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(reached);
}

TEST_F(EventTest, MultipleWaiters)
{
  Event event(*ctx_);

  int count = 0;
  auto make_waiter = [&]() -> Task<void> {
    co_await event.wait();
    count++;
  };

  auto task1 = ctx_->create_task(make_waiter());
  auto task2 = ctx_->create_task(make_waiter());
  auto task3 = ctx_->create_task(make_waiter());

  for (int i = 0; i < 10; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_EQ(count, 0);

  auto timer = node_->create_wall_timer(10ms, [&event]() { event.set(); });
  spin_until_done(task1);

  EXPECT_TRUE(task1.handle.done());
  EXPECT_TRUE(task2.handle.done());
  EXPECT_TRUE(task3.handle.done());
  EXPECT_EQ(count, 3);
}

TEST_F(EventTest, ClearAndReuse)
{
  Event event(*ctx_);

  int count = 0;
  auto coro = [&]() -> Task<void> {
    co_await event.wait();
    count++;
  };

  // First round
  auto task1 = ctx_->create_task(coro());
  auto timer1 = node_->create_wall_timer(10ms, [&event]() { event.set(); });
  spin_until_done(task1);
  EXPECT_EQ(count, 1);
  timer1->cancel();

  // Clear and reuse
  event.clear();
  auto task2 = ctx_->create_task(coro());
  for (int i = 0; i < 10; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_EQ(count, 1);  // still waiting

  auto timer2 = node_->create_wall_timer(10ms, [&event]() { event.set(); });
  spin_until_done(task2);
  EXPECT_EQ(count, 2);
}

TEST_F(EventTest, CancelDuringWait)
{
  Event event(*ctx_);

  bool was_cancelled = false;
  auto coro = [&]() -> Task<void> {
    try {
      co_await event.wait();
    } catch (const CancelledException &) {
      was_cancelled = true;
    }
  };

  auto task = coro();
  auto running = ctx_->create_task(std::move(task));

  // Spin a bit — task should be suspended
  for (int i = 0; i < 10; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(1ms);
  }

  running.cancel();
  spin_until_done(running);

  ASSERT_TRUE(running.handle.done());
  EXPECT_TRUE(was_cancelled);
}

TEST_F(EventTest, CancelDoesNotDoubleResume)
{
  Event event(*ctx_);

  bool was_cancelled = false;
  bool reached_end = false;
  auto coro = [&]() -> Task<void> {
    try {
      co_await event.wait();
    } catch (const CancelledException &) {
      was_cancelled = true;
    }
    reached_end = true;
  };

  auto task = coro();
  auto running = ctx_->create_task(std::move(task));

  // Spin a bit — task should be suspended
  for (int i = 0; i < 10; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(1ms);
  }

  running.cancel();
  spin_until_done(running);

  ASSERT_TRUE(running.handle.done());
  EXPECT_TRUE(was_cancelled);

  // Now set the event — should not cause a double-resume/crash
  event.set();

  // Spin a bit more to verify no crash
  for (int i = 0; i < 10; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_TRUE(reached_end);
}
