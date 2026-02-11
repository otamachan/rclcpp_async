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
#include <thread>

#include "rclcpp_async/rclcpp_async.hpp"

using namespace rclcpp_async;          // NOLINT(build/namespaces)
using namespace std::chrono_literals;  // NOLINT(build/namespaces)

class ChannelTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_channel_node");
    ctx_ = std::make_unique<CoContext>(*node_);
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

TEST_F(ChannelTest, SendThenNext)
{
  Channel<int> ch(*ctx_);
  ch.send(42);

  int received = 0;
  auto coro = [&]() -> Task<void> {
    auto r = co_await ch.next();
    EXPECT_TRUE(r.has_value());
    received = *r;
  };
  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_EQ(received, 42);
}

TEST_F(ChannelTest, NextThenSend)
{
  Channel<int> ch(*ctx_);

  int received = 0;
  auto coro = [&]() -> Task<void> {
    auto r = co_await ch.next();
    EXPECT_TRUE(r.has_value());
    received = *r;
  };
  auto task = ctx_->create_task(coro());

  // Spin a bit — task should be suspended
  for (int i = 0; i < 10; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_EQ(received, 0);

  ch.send(99);
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_EQ(received, 99);
}

TEST_F(ChannelTest, MultipleMessages)
{
  Channel<int> ch(*ctx_);
  ch.send(1);
  ch.send(2);
  ch.send(3);

  std::vector<int> received;
  auto coro = [&]() -> Task<void> {
    for (int i = 0; i < 3; i++) {
      auto r = co_await ch.next();
      EXPECT_TRUE(r.has_value());
      received.push_back(*r);
    }
  };
  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  ASSERT_EQ(received.size(), 3u);
  EXPECT_EQ(received[0], 1);
  EXPECT_EQ(received[1], 2);
  EXPECT_EQ(received[2], 3);
}

TEST_F(ChannelTest, CloseReturnsNullopt)
{
  Channel<int> ch(*ctx_);
  ch.close();

  bool got_nullopt = false;
  auto coro = [&]() -> Task<void> {
    auto r = co_await ch.next();
    got_nullopt = !r.has_value();
  };
  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(got_nullopt);
}

TEST_F(ChannelTest, SendFromThread)
{
  Channel<int> ch(*ctx_);

  int received = 0;
  auto coro = [&]() -> Task<void> {
    auto r = co_await ch.next();
    EXPECT_TRUE(r.has_value());
    received = *r;
  };
  auto task = ctx_->create_task(coro());

  std::thread t([&ch]() {
    std::this_thread::sleep_for(50ms);
    ch.send(77);
  });

  spin_until_done(task);
  t.join();

  ASSERT_TRUE(task.handle.done());
  EXPECT_EQ(received, 77);
}

TEST_F(ChannelTest, StreamFromThread)
{
  Channel<int> ch(*ctx_);

  std::vector<int> received;
  auto coro = [&]() -> Task<void> {
    while (true) {
      auto r = co_await ch.next();
      if (!r.has_value()) {
        break;
      }
      received.push_back(*r);
    }
  };
  auto task = ctx_->create_task(coro());

  std::thread t([&ch]() {
    for (int i = 0; i < 5; i++) {
      std::this_thread::sleep_for(10ms);
      ch.send(i);
    }
    std::this_thread::sleep_for(10ms);
    ch.close();
  });

  spin_until_done(task);
  t.join();

  ASSERT_TRUE(task.handle.done());
  ASSERT_EQ(received.size(), 5u);
  for (int i = 0; i < 5; i++) {
    EXPECT_EQ(received[i], i);
  }
}

TEST_F(ChannelTest, CancelDuringNext)
{
  Channel<int> ch(*ctx_);

  bool was_cancelled = false;
  auto coro = [&]() -> Task<void> {
    try {
      co_await ch.next();
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

TEST_F(ChannelTest, CancelFromThread)
{
  Channel<int> ch(*ctx_);

  bool was_cancelled = false;
  auto coro = [&]() -> Task<void> {
    try {
      co_await ch.next();
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

  std::thread t([&running]() {
    std::this_thread::sleep_for(50ms);
    running.cancel();
  });

  spin_until_done(running);
  t.join();

  ASSERT_TRUE(running.handle.done());
  EXPECT_TRUE(was_cancelled);
}
