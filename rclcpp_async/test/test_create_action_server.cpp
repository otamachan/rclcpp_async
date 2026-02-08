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
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vector>

#include "rclcpp_async/rclcpp_async.hpp"

using namespace rclcpp_async;          // NOLINT(build/namespaces)
using namespace std::chrono_literals;  // NOLINT(build/namespaces)
using Fibonacci = example_interfaces::action::Fibonacci;

class CreateActionServerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_action_server_node");
    ctx_ = std::make_unique<CoContext>(node_);
    executor_.add_node(node_);
    action_client_ = rclcpp_action::create_client<Fibonacci>(node_, "test_coro_action");
  }

  void TearDown() override
  {
    action_server_.reset();
    action_client_.reset();
    ctx_.reset();
    node_.reset();
  }

  void spin_until_done(Task<void> & task, std::chrono::seconds timeout = 10s)
  {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!task.handle.done() && std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(1ms);
    }
  }

  void wait_for_server()
  {
    for (int i = 0; i < 50; i++) {
      executor_.spin_some();
      std::this_thread::sleep_for(10ms);
      if (action_client_->action_server_is_ready()) {
        break;
      }
    }
    ASSERT_TRUE(action_client_->action_server_is_ready());
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<CoContext> ctx_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};

TEST_F(CreateActionServerTest, BasicExecution)
{
  action_server_ = ctx_->create_action_server<Fibonacci>(
    "test_coro_action", [this](GoalContext<Fibonacci> goal) -> Task<Fibonacci::Result> {
      Fibonacci::Feedback feedback;
      feedback.sequence = {0, 1};

      for (int i = 2; i < goal.goal().order; i++) {
        if (goal.is_canceling()) {
          break;
        }
        auto n = feedback.sequence.size();
        feedback.sequence.push_back(feedback.sequence[n - 1] + feedback.sequence[n - 2]);
        goal.publish_feedback(feedback);
        co_await ctx_->sleep(10ms);
      }

      Fibonacci::Result result;
      result.sequence = feedback.sequence;
      co_return result;
    });

  wait_for_server();

  std::vector<int32_t> final_sequence;
  int feedback_count = 0;

  auto coro = [&]() -> Task<void> {
    Fibonacci::Goal goal;
    goal.order = 6;
    auto goal_result = co_await ctx_->send_goal<Fibonacci>(action_client_, goal);
    EXPECT_TRUE(goal_result.ok());
    if (!goal_result.ok()) {
      co_return;
    }

    auto stream = *goal_result.value;
    while (true) {
      auto r = co_await stream->next();
      if (!r.ok() || !r.value->has_value()) {
        break;
      }
      feedback_count++;
    }

    auto wrapped = stream->result();
    EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
    final_sequence = wrapped.result->sequence;
  };

  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_GE(feedback_count, 1);
  // Fibonacci(6): 0, 1, 1, 2, 3, 5
  ASSERT_EQ(final_sequence.size(), 6u);
  EXPECT_EQ(final_sequence[0], 0);
  EXPECT_EQ(final_sequence[1], 1);
  EXPECT_EQ(final_sequence[5], 5);
}

TEST_F(CreateActionServerTest, CancelDuringExecution)
{
  action_server_ = ctx_->create_action_server<Fibonacci>(
    "test_coro_action", [this](GoalContext<Fibonacci> goal) -> Task<Fibonacci::Result> {
      Fibonacci::Feedback feedback;
      feedback.sequence = {0, 1};

      for (int i = 2; i < goal.goal().order; i++) {
        if (goal.is_canceling()) {
          break;
        }
        auto n = feedback.sequence.size();
        feedback.sequence.push_back(feedback.sequence[n - 1] + feedback.sequence[n - 2]);
        goal.publish_feedback(feedback);
        co_await ctx_->sleep(100ms);
      }

      Fibonacci::Result result;
      result.sequence = feedback.sequence;
      co_return result;
    });

  wait_for_server();

  int feedback_count = 0;
  bool goal_cancelled = false;

  auto coro = [&]() -> Task<void> {
    Fibonacci::Goal goal;
    goal.order = 50;
    auto goal_result = co_await ctx_->send_goal<Fibonacci>(action_client_, goal);
    EXPECT_TRUE(goal_result.ok());
    if (!goal_result.ok()) {
      co_return;
    }

    auto stream = *goal_result.value;
    while (true) {
      auto r = co_await stream->next();
      if (!r.ok() || !r.value->has_value()) {
        break;
      }
      feedback_count++;
      if (feedback_count >= 2) {
        co_await stream->cancel_goal();
      }
    }

    auto wrapped = stream->result();
    goal_cancelled = (wrapped.code == rclcpp_action::ResultCode::CANCELED);
  };

  auto task = ctx_->create_task(coro());
  spin_until_done(task, 15s);

  ASSERT_TRUE(task.handle.done());
  EXPECT_GE(feedback_count, 2);
  EXPECT_TRUE(goal_cancelled);
}

TEST_F(CreateActionServerTest, AbortDuringExecution)
{
  action_server_ = ctx_->create_action_server<Fibonacci>(
    "test_coro_action", [this](GoalContext<Fibonacci> goal) -> Task<Fibonacci::Result> {
      if (goal.goal().order < 0) {
        goal.abort();
        Fibonacci::Result result;
        co_return result;
      }

      Fibonacci::Feedback feedback;
      feedback.sequence = {0, 1};

      for (int i = 2; i < goal.goal().order; i++) {
        auto n = feedback.sequence.size();
        feedback.sequence.push_back(feedback.sequence[n - 1] + feedback.sequence[n - 2]);
        goal.publish_feedback(feedback);
        co_await ctx_->sleep(10ms);
      }

      Fibonacci::Result result;
      result.sequence = feedback.sequence;
      co_return result;
    });

  wait_for_server();

  bool goal_aborted = false;

  auto coro = [&]() -> Task<void> {
    Fibonacci::Goal goal;
    goal.order = -1;
    auto goal_result = co_await ctx_->send_goal<Fibonacci>(action_client_, goal);
    EXPECT_TRUE(goal_result.ok());
    if (!goal_result.ok()) {
      co_return;
    }

    auto stream = *goal_result.value;
    while (true) {
      auto r = co_await stream->next();
      if (!r.ok() || !r.value->has_value()) {
        break;
      }
    }

    auto wrapped = stream->result();
    goal_aborted = (wrapped.code == rclcpp_action::ResultCode::ABORTED);
  };

  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(goal_aborted);
}
