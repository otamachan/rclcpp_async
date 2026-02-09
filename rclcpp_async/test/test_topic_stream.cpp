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
#include <std_msgs/msg/string.hpp>

#include "rclcpp_async/rclcpp_async.hpp"

using namespace rclcpp_async;          // NOLINT(build/namespaces)
using namespace std::chrono_literals;  // NOLINT(build/namespaces)
using StringMsg = std_msgs::msg::String;

class TopicStreamTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_topic_node");
    ctx_ = std::make_unique<CoContext>(node_);
    executor_.add_node(node_);
    pub_ = node_->create_publisher<StringMsg>("test_topic", 10);
  }

  void TearDown() override
  {
    pub_.reset();
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

  void publish(const std::string & data)
  {
    auto msg = std::make_unique<StringMsg>();
    msg->data = data;
    pub_->publish(std::move(msg));
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<CoContext> ctx_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<StringMsg>::SharedPtr pub_;
};

TEST_F(TopicStreamTest, SubscribeAndReceiveMessage)
{
  std::string received;

  auto coro = [&]() -> Task<void> {
    auto stream = ctx_->subscribe<StringMsg>("test_topic", 10);
    auto msg = co_await stream->next();
    if (msg.has_value()) {
      received = (*msg)->data;
    }
  };

  auto task = ctx_->create_task(coro());

  // Let subscription establish
  for (int i = 0; i < 30; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  publish("hello_topic");
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_EQ(received, "hello_topic");
}

TEST_F(TopicStreamTest, ReceiveMultipleMessages)
{
  std::vector<std::string> received;

  auto coro = [&]() -> Task<void> {
    auto stream = ctx_->subscribe<StringMsg>("test_topic", 10);
    for (int i = 0; i < 3; i++) {
      auto msg = co_await stream->next();
      if (msg.has_value()) {
        received.push_back((*msg)->data);
      }
    }
  };

  auto task = ctx_->create_task(coro());

  for (int i = 0; i < 30; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  publish("msg1");
  publish("msg2");
  publish("msg3");
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  ASSERT_EQ(received.size(), 3u);
  EXPECT_EQ(received[0], "msg1");
  EXPECT_EQ(received[1], "msg2");
  EXPECT_EQ(received[2], "msg3");
}

TEST_F(TopicStreamTest, CloseReturnsNullopt)
{
  bool got_nullopt = false;

  auto coro = [&]() -> Task<void> {
    auto stream = ctx_->subscribe<StringMsg>("test_topic", 10);
    stream->close();
    auto msg = co_await stream->next();
    got_nullopt = !msg.has_value();
  };

  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(got_nullopt);
}

TEST_F(TopicStreamTest, CancelDuringNext)
{
  bool was_cancelled = false;

  auto coro = [&]() -> Task<void> {
    auto stream = ctx_->subscribe<StringMsg>("test_topic", 10);
    try {
      co_await stream->next();
    } catch (const CancelledException &) {
      was_cancelled = true;
    }
  };

  auto task = coro();
  auto running = ctx_->create_task(std::move(task));

  // Let subscription establish and coroutine suspend
  for (int i = 0; i < 30; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  running.cancel();
  spin_until_done(running);

  ASSERT_TRUE(running.handle.done());
  EXPECT_TRUE(was_cancelled);
}
