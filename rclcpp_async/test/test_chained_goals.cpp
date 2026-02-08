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

// Regression test for rclcpp_action deadlock when chaining send_goal.
// See: https://github.com/ros2/rclcpp/issues/2796
//
// In Jazzy, rclcpp_action::ClientBase::handle_goal_response() holds
// goal_requests_mutex while invoking the user's goal_response_callback.
// If the coroutine resumed from that callback immediately calls
// async_send_goal again, it will attempt to re-lock the same
// non-reentrant mutex -> deadlock.

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

class ChainedGoalsTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_chained_goals_node");
    ctx_ = std::make_unique<CoContext>(node_);
    executor_.add_node(node_);
    action_client_ = rclcpp_action::create_client<Fibonacci>(node_, "test_chained_action");
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

  void create_fast_server()
  {
    action_server_ = rclcpp_action::create_server<Fibonacci>(
      node_, "test_chained_action",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [](const std::shared_ptr<GoalHandle> goal_handle) {
        // Complete immediately with a minimal result
        std::thread([goal_handle]() {
          auto result = std::make_shared<Fibonacci::Result>();
          result->sequence = {0, 1, 1};
          goal_handle->succeed(result);
        }).detach();
      });
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<CoContext> ctx_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};

// Chain two send_goal calls on the same client.
// On Jazzy with the unfixed rclcpp_action, this deadlocks because
// goal_response_callback is invoked while holding goal_requests_mutex,
// and the resumed coroutine's second async_send_goal tries to re-lock it.
// Chain two send_goal calls with stream drain in between.
// The intervening co_await on stream->next() releases the mutex,
// so this should NOT deadlock even on unfixed Jazzy.
TEST_F(ChainedGoalsTest, TwoSequentialGoalsWithDrain)
{
  create_fast_server();
  wait_for_server();

  int goals_completed = 0;

  auto coro = [&]() -> Task<void> {
    Fibonacci::Goal goal;
    goal.order = 3;

    // First goal
    auto r1 = co_await ctx_->send_goal<Fibonacci>(action_client_, goal);
    EXPECT_TRUE(r1.ok()) << "first goal failed";
    if (r1.ok()) {
      auto stream1 = *r1.value;
      while (true) {
        auto ev = co_await stream1->next();
        if (!ev.ok() || !ev.value->has_value()) {
          break;
        }
      }
      goals_completed++;
    }

    // Second goal — safe because stream drain above suspended the coroutine,
    // allowing goal_response_callback to return and release the mutex.
    auto r2 = co_await ctx_->send_goal<Fibonacci>(action_client_, goal);
    EXPECT_TRUE(r2.ok()) << "second goal failed";
    if (r2.ok()) {
      auto stream2 = *r2.value;
      while (true) {
        auto ev = co_await stream2->next();
        if (!ev.ok() || !ev.value->has_value()) {
          break;
        }
      }
      goals_completed++;
    }
  };

  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done()) << "task did not complete — likely deadlocked";
  EXPECT_EQ(goals_completed, 2);
}

// Immediately chain two send_goal calls WITHOUT draining the first stream.
// This triggers the Jazzy deadlock: goal_response_callback holds
// goal_requests_mutex, resumes the coroutine via ctx.resume(h), and the
// coroutine immediately calls async_send_goal which re-locks the same mutex.
TEST_F(ChainedGoalsTest, ImmediateChainDeadlockReproduction)
{
  create_fast_server();
  wait_for_server();

  int goals_accepted = 0;

  auto coro = [&]() -> Task<void> {
    Fibonacci::Goal goal;
    goal.order = 3;

    // First goal — do NOT drain the stream, immediately send second goal
    auto r1 = co_await ctx_->send_goal<Fibonacci>(action_client_, goal);
    if (r1.ok()) {
      goals_accepted++;
    }

    // Second goal — this resumes directly inside goal_response_callback
    // of the first goal. On unfixed Jazzy, async_send_goal will try to
    // lock goal_requests_mutex which is already held -> DEADLOCK.
    auto r2 = co_await ctx_->send_goal<Fibonacci>(action_client_, goal);
    if (r2.ok()) {
      goals_accepted++;
    }
  };

  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done()) << "task did not complete — likely deadlocked";
  EXPECT_EQ(goals_accepted, 2);
}
