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

#include "rclcpp_async/rclcpp_async.hpp"

using namespace rclcpp_async;          // NOLINT(build/namespaces)
using namespace std::chrono_literals;  // NOLINT(build/namespaces)
using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

class GoalStreamTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_goal_node");
    ctx_ = std::make_unique<CoContext>(node_);
    action_client_ = rclcpp_action::create_client<Fibonacci>(node_, "test_goal_action");
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

  void create_accepting_server(int feedback_count = 3, int feedback_interval_ms = 200)
  {
    action_server_ = rclcpp_action::create_server<Fibonacci>(
      node_, "test_goal_action",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [feedback_count, feedback_interval_ms](const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread([goal_handle, feedback_count, feedback_interval_ms]() {
          std::this_thread::sleep_for(500ms);

          auto feedback = std::make_shared<Fibonacci::Feedback>();
          feedback->sequence = {0, 1};
          for (int i = 0; i < feedback_count; i++) {
            if (goal_handle->is_canceling()) {
              auto result = std::make_shared<Fibonacci::Result>();
              result->sequence = feedback->sequence;
              goal_handle->canceled(result);
              return;
            }
            auto n = feedback->sequence.size();
            feedback->sequence.push_back(feedback->sequence[n - 1] + feedback->sequence[n - 2]);
            goal_handle->publish_feedback(feedback);
            std::this_thread::sleep_for(std::chrono::milliseconds(feedback_interval_ms));
          }

          if (goal_handle->is_canceling()) {
            auto result = std::make_shared<Fibonacci::Result>();
            result->sequence = feedback->sequence;
            goal_handle->canceled(result);
            return;
          }

          auto result = std::make_shared<Fibonacci::Result>();
          result->sequence = feedback->sequence;
          goal_handle->succeed(result);
        }).detach();
      });
  }

  void create_rejecting_server()
  {
    action_server_ = rclcpp_action::create_server<Fibonacci>(
      node_, "test_goal_action",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::REJECT;
      },
      [](const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [](const std::shared_ptr<GoalHandle>) {});
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<CoContext> ctx_;
  rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};

TEST_F(GoalStreamTest, SendGoalAndReceiveResult)
{
  create_accepting_server();

  for (int i = 0; i < 50; i++) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(10ms);
    if (action_client_->action_server_is_ready()) {
      break;
    }
  }
  ASSERT_TRUE(action_client_->action_server_is_ready());

  std::vector<int32_t> final_sequence;
  bool got_feedback = false;

  auto coro = [&]() -> Task<void> {
    Fibonacci::Goal goal;
    goal.order = 5;
    auto goal_result = co_await ctx_->send_goal<Fibonacci>(action_client_, goal);
    if (!goal_result.ok()) {
      co_return;
    }

    auto stream = *goal_result.value;
    while (true) {
      auto event = co_await stream->next();
      if (!event.has_value()) {
        break;
      }
      got_feedback = true;
    }
    final_sequence = stream->result().result->sequence;
  };

  auto task = ctx_->create_task(coro());
  spin_until_done(task, 10s);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(got_feedback);
  ASSERT_FALSE(final_sequence.empty());
  EXPECT_EQ(final_sequence.back(), 3);
}

TEST_F(GoalStreamTest, GoalRejected)
{
  create_rejecting_server();

  for (int i = 0; i < 50; i++) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(10ms);
    if (action_client_->action_server_is_ready()) {
      break;
    }
  }
  ASSERT_TRUE(action_client_->action_server_is_ready());

  bool was_error = false;
  std::string error_msg;

  auto coro = [&]() -> Task<void> {
    Fibonacci::Goal goal;
    goal.order = 5;
    auto goal_result = co_await ctx_->send_goal<Fibonacci>(action_client_, goal);
    was_error = !goal_result.ok();
    error_msg = goal_result.error_msg;
  };

  auto task = ctx_->create_task(coro());
  spin_until_done(task, 10s);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(was_error);
  EXPECT_EQ(error_msg, "goal rejected");
}

TEST_F(GoalStreamTest, CancelGoal)
{
  // Server sends many feedbacks with delay, giving time to cancel mid-execution
  create_accepting_server(20, 300);

  for (int i = 0; i < 50; i++) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(10ms);
    if (action_client_->action_server_is_ready()) {
      break;
    }
  }
  ASSERT_TRUE(action_client_->action_server_is_ready());

  int feedback_count = 0;
  bool goal_cancelled = false;

  auto coro = [&]() -> Task<void> {
    Fibonacci::Goal goal;
    goal.order = 20;
    auto goal_result = co_await ctx_->send_goal<Fibonacci>(action_client_, goal);
    if (!goal_result.ok()) {
      EXPECT_TRUE(false) << "goal was not accepted";
      co_return;
    }

    auto stream = *goal_result.value;

    // Receive a few feedbacks, then cancel
    while (true) {
      auto event = co_await stream->next();
      if (!event.has_value()) {
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

TEST_F(GoalStreamTest, AlreadyCancelledIsImmediate)
{
  create_accepting_server();

  for (int i = 0; i < 50; i++) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(10ms);
    if (action_client_->action_server_is_ready()) {
      break;
    }
  }

  bool was_cancelled = false;

  auto coro = [&]() -> Task<void> {
    Fibonacci::Goal goal;
    goal.order = 5;
    auto goal_result = co_await ctx_->send_goal<Fibonacci>(action_client_, goal);
    was_cancelled = goal_result.cancelled();
  };

  auto task = coro();
  task.cancel();
  auto running = ctx_->create_task(std::move(task));
  spin_until_done(running, 10s);

  ASSERT_TRUE(running.handle.done());
  EXPECT_TRUE(was_cancelled);
}
