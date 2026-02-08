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
#include <example_interfaces/action/fibonacci.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "rclcpp_async/rclcpp_async.hpp"

using namespace rclcpp_async;          // NOLINT(build/namespaces)
using namespace std::chrono_literals;  // NOLINT(build/namespaces)
using Fibonacci = example_interfaces::action::Fibonacci;

class WaitForActionTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_wfa_node");
    ctx_ = std::make_unique<CoContext>(node_);
    action_client_ = rclcpp_action::create_client<Fibonacci>(node_, "test_wfa_action");
  }

  void TearDown() override
  {
    action_server_.reset();
    action_client_.reset();
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

  void create_action_server()
  {
    using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;
    action_server_ = rclcpp_action::create_server<Fibonacci>(
      node_, "test_wfa_action",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [](const std::shared_ptr<GoalHandle> goal_handle) {
        auto result = std::make_shared<Fibonacci::Result>();
        result->sequence = {0, 1, 1};
        goal_handle->succeed(result);
      });
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<CoContext> ctx_;
  rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};

TEST_F(WaitForActionTest, ServerAlreadyReady)
{
  create_action_server();
  for (int i = 0; i < 50; i++) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(10ms);
    if (action_client_->action_server_is_ready()) {
      break;
    }
  }

  Result<void> result;
  auto coro = [&]() -> Task<void> { result = co_await ctx_->wait_for_action(action_client_, 1s); };
  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(result.ok());
}

TEST_F(WaitForActionTest, ServerBecomesReady)
{
  Result<void> result;
  auto coro = [&]() -> Task<void> { result = co_await ctx_->wait_for_action(action_client_, 5s); };
  auto task = ctx_->create_task(coro());

  for (int i = 0; i < 10; i++) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(10ms);
  }
  create_action_server();
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(result.ok());
}

TEST_F(WaitForActionTest, Timeout)
{
  Result<void> result;
  auto coro = [&]() -> Task<void> {
    result = co_await ctx_->wait_for_action(action_client_, 200ms);
  };
  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(result.timeout());
}

TEST_F(WaitForActionTest, Cancel)
{
  Result<void> result;
  auto coro = [&]() -> Task<void> { result = co_await ctx_->wait_for_action(action_client_, 10s); };
  auto task = ctx_->create_task(coro());

  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(10ms);
  rclcpp::spin_some(node_);

  task.cancel();
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(result.cancelled());
}
