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

class TaskDestroyTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_task_destroy_node");
    ctx_ = std::make_unique<CoContext>(*node_);
    executor_.add_node(node_);
    action_client_ = rclcpp_action::create_client<Fibonacci>(node_, "test_destroy_action");
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
      executor_.spin_some();
      std::this_thread::sleep_for(1ms);
    }
  }

  void create_feedback_server(int count = 30, int interval_ms = 100)
  {
    action_server_ = rclcpp_action::create_server<Fibonacci>(
      node_, "test_destroy_action",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [count, interval_ms](const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread([goal_handle, count, interval_ms]() {
          std::this_thread::sleep_for(200ms);
          auto feedback = std::make_shared<Fibonacci::Feedback>();
          feedback->sequence = {0, 1};
          for (int i = 0; i < count; i++) {
            if (goal_handle->is_canceling()) {
              auto result = std::make_shared<Fibonacci::Result>();
              result->sequence = feedback->sequence;
              goal_handle->canceled(result);
              return;
            }
            auto n = feedback->sequence.size();
            feedback->sequence.push_back(feedback->sequence[n - 1] + feedback->sequence[n - 2]);
            goal_handle->publish_feedback(feedback);
            std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
          }
          auto result = std::make_shared<Fibonacci::Result>();
          result->sequence = feedback->sequence;
          goal_handle->succeed(result);
        }).detach();
      });
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

TEST_F(TaskDestroyTest, DestroyWhileSuspendedOnTimer)
{
  bool outer_done = false;

  auto coro = [&]() -> Task<void> {
    {
      auto inner = ctx_->create_task([this]() -> Task<void> {
        auto timer = ctx_->create_timer(50ms);
        while (true) {
          co_await timer->next();
        }
      });
      co_await ctx_->sleep(200ms);
    }
    co_await ctx_->sleep(500ms);
    outer_done = true;
  };

  auto task = ctx_->create_task(coro());
  spin_until_done(task, 10s);
  EXPECT_TRUE(outer_done);
}

TEST_F(TaskDestroyTest, CancelAndDestroyWhileSuspendedOnTimer)
{
  bool outer_done = false;

  auto coro = [&]() -> Task<void> {
    {
      auto inner = ctx_->create_task([this]() -> Task<void> {
        auto timer = ctx_->create_timer(10s);
        try {
          co_await timer->next();
        } catch (const CancelledException &) {
        }
      });
      co_await ctx_->sleep(50ms);
      inner.cancel();
    }
    co_await ctx_->sleep(200ms);
    outer_done = true;
  };

  auto task = ctx_->create_task(coro());
  spin_until_done(task, 10s);
  EXPECT_TRUE(outer_done);
}

TEST_F(TaskDestroyTest, DestroyGoalTaskWhileAwaitingFeedback)
{
  create_feedback_server(30, 100);
  wait_for_server();

  bool outer_done = false;

  auto coro = [&]() -> Task<void> {
    {
      auto inner = ctx_->create_task([this]() -> Task<void> {
        Fibonacci::Goal goal;
        goal.order = 30;
        auto goal_result = co_await ctx_->send_goal<Fibonacci>(action_client_, goal);
        if (!goal_result.ok()) {
          co_return;
        }
        auto stream = *goal_result.value;
        while (true) {
          auto feedback = co_await stream->next();
          if (!feedback.has_value()) {
            break;
          }
        }
      });
      co_await ctx_->sleep(1s);
    }
    co_await ctx_->sleep(2s);
    outer_done = true;
  };

  auto task = ctx_->create_task(coro());
  spin_until_done(task, 15s);
  EXPECT_TRUE(outer_done);
}
