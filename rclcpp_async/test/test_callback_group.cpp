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

#include <atomic>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

#include "rclcpp_async/rclcpp_async.hpp"

using namespace rclcpp_async;          // NOLINT(build/namespaces)
using namespace std::chrono_literals;  // NOLINT(build/namespaces)

class CallbackGroupTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_callback_group_node");
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    ctx_ = std::make_unique<CoContext>(*node_, group_);
    executor_.add_node(node_);
  }

  void TearDown() override
  {
    ctx_.reset();
    group_.reset();
    node_.reset();
  }

  // Every callback of the group runs this; two of them running at once means
  // the group did not serialize them.
  void Enter()
  {
    if (active_.fetch_add(1) != 0) {
      overlapped_ = true;
    }
    std::this_thread::sleep_for(200us);
    active_.fetch_sub(1);
  }

  void SpinUntil(const std::atomic<bool> & done, std::chrono::seconds timeout = 10s)
  {
    std::thread spinner([this]() { executor_.spin(); });
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!done && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(1ms);
    }
    executor_.cancel();
    spinner.join();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::unique_ptr<CoContext> ctx_;
  rclcpp::executors::MultiThreadedExecutor executor_{rclcpp::ExecutorOptions(), 4};
  std::atomic<int> active_{0};
  std::atomic<bool> overlapped_{false};
};

TEST_F(CallbackGroupTest, ContextReportsItsGroup) { EXPECT_EQ(ctx_->callback_group(), group_); }

TEST_F(CallbackGroupTest, CoroutineResumesSerializedWithItsGroup)
{
  auto timer = node_->create_wall_timer(1ms, [this]() { Enter(); }, group_);
  std::atomic<bool> done{false};
  auto task = ctx_->create_task([this, &done]() -> Task<void> {
    for (int i = 0; i < 200; ++i) {
      co_await ctx_->sleep(1ms);
      Enter();
    }
    auto ticks = ctx_->create_timer(1ms);
    for (int i = 0; i < 100; ++i) {
      co_await ticks->next();
      Enter();
    }
    done = true;
  });

  SpinUntil(done);
  EXPECT_TRUE(done);
  EXPECT_FALSE(overlapped_);
}

TEST_F(CallbackGroupTest, ServiceHandlerRunsInTheGroup)
{
  auto timer = node_->create_wall_timer(1ms, [this]() { Enter(); }, group_);
  auto service = ctx_->create_service<std_srvs::srv::Trigger>(
    "test_callback_group_trigger",
    [this](std_srvs::srv::Trigger::Request::SharedPtr) -> Task<std_srvs::srv::Trigger::Response> {
      Enter();
      co_await ctx_->sleep(1ms);
      Enter();
      std_srvs::srv::Trigger::Response response;
      response.success = true;
      co_return response;
    });
  auto client_group = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto client = node_->create_client<std_srvs::srv::Trigger>(
    "test_callback_group_trigger", rclcpp::ServicesQoS(), client_group);

  std::atomic<int> answered{0};
  std::atomic<bool> done{false};
  std::thread caller([&]() {
    client->wait_for_service(5s);
    for (int i = 0; i < 50; ++i) {
      auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      if (future.wait_for(2s) == std::future_status::ready && future.get()->success) {
        ++answered;
      }
    }
    done = true;
  });

  SpinUntil(done);
  caller.join();
  EXPECT_EQ(answered, 50);
  EXPECT_FALSE(overlapped_);
}
