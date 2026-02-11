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

class TimerStreamTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_timer_stream_node");
    ctx_ = std::make_unique<CoContext>(*node_);
    executor_.add_node(node_);
  }

  void TearDown() override
  {
    ctx_.reset();
    node_.reset();
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(1ms);
    }
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

TEST_F(TimerStreamTest, BasicNext)
{
  int count = 0;
  bool done = false;

  auto task = ctx_->create_task([this, &count, &done]() -> Task<void> {
    auto timer = ctx_->create_timer(50ms);
    for (int i = 0; i < 5; i++) {
      co_await timer->next();
      count++;
    }
    timer->cancel();
    done = true;
  });

  spin_for(500ms);

  EXPECT_TRUE(done);
  EXPECT_EQ(count, 5);
}

TEST_F(TimerStreamTest, Cancel)
{
  int count = 0;
  bool done = false;

  std::shared_ptr<TimerStream> timer;
  auto task = ctx_->create_task([this, &count, &done, &timer]() -> Task<void> {
    timer = ctx_->create_timer(50ms);
    while (true) {
      co_await timer->next();
      count++;
      if (count >= 3) {
        break;
      }
    }
    timer->cancel();
    done = true;
  });

  spin_for(500ms);

  EXPECT_TRUE(done);
  EXPECT_EQ(count, 3);
}

TEST_F(TimerStreamTest, PendingAccumulation)
{
  // Timer fires while coroutine is busy with sleep — pending should accumulate
  int tick_count = 0;
  bool done = false;

  auto task = ctx_->create_task([this, &tick_count, &done]() -> Task<void> {
    auto timer = ctx_->create_timer(30ms);

    // First, wait for timer to fire
    co_await timer->next();
    tick_count++;

    // Sleep longer than one timer period, so pending accumulates
    co_await ctx_->sleep(100ms);

    // next() should return immediately (pending > 0)
    co_await timer->next();
    tick_count++;

    timer->cancel();
    done = true;
  });

  spin_for(500ms);

  EXPECT_TRUE(done);
  EXPECT_EQ(tick_count, 2);
}

TEST_F(TimerStreamTest, CancelDuringNext)
{
  bool was_cancelled = false;

  auto coro = [&]() -> Task<void> {
    auto timer = ctx_->create_timer(10s);  // Very long period — will be suspended
    try {
      co_await timer->next();
    } catch (const CancelledException &) {
      was_cancelled = true;
    }
    timer->cancel();
  };

  auto task = coro();
  auto running = ctx_->create_task(std::move(task));

  // Let coroutine suspend on next()
  for (int i = 0; i < 10; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(1ms);
  }

  running.cancel();
  spin_until_done(running);

  ASSERT_TRUE(running.handle.done());
  EXPECT_TRUE(was_cancelled);
}
