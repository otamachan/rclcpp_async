// Copyright 2026 Tamaki Nishino
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
#include <example_interfaces/action/fibonacci.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>

#include "rclcpp_async/rclcpp_async.hpp"

using namespace rclcpp_async;          // NOLINT(build/namespaces)
using namespace std::chrono_literals;  // NOLINT(build/namespaces)
using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

// Regression test for a use-after-free in SendGoalAwaiter: if the awaiting
// coroutine is cancelled between async_send_goal() and the arrival of the
// server's goal response, the goal_response_callback must not touch the
// (already-destroyed) awaiter/coroutine frame when the response finally
// arrives.
class SendGoalCancelTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    client_node_ = std::make_shared<rclcpp::Node>("test_send_goal_cancel_client");
    server_node_ = std::make_shared<rclcpp::Node>("test_send_goal_cancel_server");
    ctx_ = std::make_unique<CoContext>(*client_node_);
    client_executor_.add_node(client_node_);
    server_executor_.add_node(server_node_);
    action_client_ =
      rclcpp_action::create_client<Fibonacci>(client_node_, "test_send_goal_cancel_action");
  }

  void TearDown() override
  {
    server_thread_stop_ = true;
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    action_server_.reset();
    action_client_.reset();
    ctx_.reset();
    client_node_.reset();
    server_node_.reset();
  }

  // Stand up an action server whose goal_callback sleeps for `delay` before
  // returning ACCEPT. The goal_callback runs on the server's executor
  // thread; by spinning the server on its own thread, the client-side
  // executor is free to process cancellation during that window.
  void start_server_with_delayed_accept(std::chrono::milliseconds delay)
  {
    action_server_ = rclcpp_action::create_server<Fibonacci>(
      server_node_, "test_send_goal_cancel_action",
      [delay](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        std::this_thread::sleep_for(delay);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [](const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread([goal_handle]() {
          auto result = std::make_shared<Fibonacci::Result>();
          result->sequence = {0, 1};
          goal_handle->succeed(result);
        }).detach();
      });

    server_thread_ = std::thread([this]() {
      while (!server_thread_stop_) {
        server_executor_.spin_some();
        std::this_thread::sleep_for(1ms);
      }
    });
  }

  void wait_for_server()
  {
    for (int i = 0; i < 100; i++) {
      client_executor_.spin_some();
      std::this_thread::sleep_for(10ms);
      if (action_client_->action_server_is_ready()) {
        break;
      }
    }
    ASSERT_TRUE(action_client_->action_server_is_ready());
  }

  void spin_client_for(std::chrono::milliseconds duration)
  {
    auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      client_executor_.spin_some();
      std::this_thread::sleep_for(1ms);
    }
  }

  void spin_client_until_done(Task<void> & task, std::chrono::milliseconds timeout)
  {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!task.handle.done() && std::chrono::steady_clock::now() < deadline) {
      client_executor_.spin_some();
      std::this_thread::sleep_for(1ms);
    }
  }

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Node::SharedPtr server_node_;
  std::unique_ptr<CoContext> ctx_;
  rclcpp::executors::SingleThreadedExecutor client_executor_;
  rclcpp::executors::SingleThreadedExecutor server_executor_;
  std::thread server_thread_;
  std::atomic<bool> server_thread_stop_{false};
  rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};

TEST_F(SendGoalCancelTest, CancelWhileAwaitingGoalResponseDoesNotCrash)
{
  // Server delays its ACCEPT decision by 300ms. The client cancels before
  // that window elapses, so the goal_response arrives after the awaiting
  // coroutine has already unwound with CancelledException.
  start_server_with_delayed_accept(300ms);
  wait_for_server();

  bool cancelled_caught = false;
  auto coro = [&]() -> Task<void> {
    Fibonacci::Goal goal;
    goal.order = 6;
    try {
      co_await ctx_->send_goal<Fibonacci>(action_client_, goal);
    } catch (const CancelledException &) {
      cancelled_caught = true;
    }
  };

  {
    auto task = ctx_->create_task(coro());

    // Let the goal request reach the server (which is now sleeping inside
    // goal_callback). The server thread spins independently.
    spin_client_for(50ms);

    // Cancel while the server is still sleeping — no goal_response yet.
    task.cancel();

    spin_client_until_done(task, 500ms);
    ASSERT_TRUE(task.handle.done());
    EXPECT_TRUE(cancelled_caught);
    // task destructor here frees the coroutine frame that holds the awaiter.
  }

  // After the frame is gone, the server's belated goal_response arrives and
  // the rclcpp_action::Client still holds a reference-captured callback into
  // (pre-fix) freed memory. Keep the client executor alive well past the
  // server's delay so the callback fires. Pre-fix this dereferences the
  // destroyed awaiter (heap-use-after-free under ASan; SIGSEGV in the wild).
  spin_client_for(1000ms);

  SUCCEED();
}
