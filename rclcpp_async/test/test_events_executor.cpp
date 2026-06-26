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

// Regression test for DrainWaitable under events-based executors.
//
// CoContext::post() defers work onto a DrainWaitable guard condition. Events
// executors (e.g. EventsCBGExecutor) never poll the wait set; they rely on
// Waitable::set_on_ready_callback to be notified. When that hook was a no-op,
// posted resumptions were never executed, so a coroutine suspended in send_goal
// (whose goal_response_callback resumes via post()) hung forever.

#include <gtest/gtest.h>

// EventsCBGExecutor is available on jazzy and lyrical (rclcpp >= 32), but not in
// the frozen rolling-on-noble image (rclcpp 31.x). Skip the test where absent.
#if __has_include(<rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp>)

#include <chrono>
#include <example_interfaces/action/fibonacci.hpp>
#include <memory>
#include <rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>

#include "rclcpp_async/rclcpp_async.hpp"

using namespace rclcpp_async;          // NOLINT(build/namespaces)
using namespace std::chrono_literals;  // NOLINT(build/namespaces)
using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;
using rclcpp::executors::EventsCBGExecutor;

class EventsCBGExecutorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_events_executor_node");
    ctx_ = std::make_unique<CoContext>(*node_);
    executor_.add_node(node_);
    action_client_ = rclcpp_action::create_client<Fibonacci>(node_, "test_events_action");
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
      node_, "test_events_action",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [](const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread([goal_handle]() {
          auto result = std::make_shared<Fibonacci::Result>();
          result->sequence = {0, 1, 1};
          goal_handle->succeed(result);
        }).detach();
      });
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<CoContext> ctx_;
  // Single-threaded EventsCBGExecutor, matching the configuration reported in
  // https://github.com/otamachan/rclcpp_async/pull/53.
  EventsCBGExecutor executor_{rclcpp::ExecutorOptions{}, 1};
  rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};

// send_goal resumes the coroutine via CoContext::post(). Under an events
// executor this only works if DrainWaitable forwards its guard condition to
// set_on_ready_callback. Without the fix the coroutine never resumes and the
// task never completes (times out).
TEST_F(EventsCBGExecutorTest, SendGoalResumesUnderEventsCBGExecutor)
{
  create_fast_server();
  wait_for_server();

  bool stream_drained = false;

  auto coro = [&]() -> Task<void> {
    Fibonacci::Goal goal;
    goal.order = 3;

    auto r = co_await ctx_->send_goal<Fibonacci>(action_client_, goal);
    EXPECT_TRUE(r.ok()) << "send_goal failed";
    if (r.ok()) {
      auto stream = *r.value;
      while (true) {
        auto ev = co_await stream->next();
        if (!ev.has_value()) {
          break;
        }
      }
      stream_drained = true;
    }
  };

  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done()) << "task did not complete — post() never resumed";
  EXPECT_TRUE(stream_drained);
}

#else  // EventsCBGExecutor not available (rolling on noble)

TEST(EventsCBGExecutorTest, SendGoalResumesUnderEventsCBGExecutor)
{
  GTEST_SKIP() << "EventsCBGExecutor not available in this distro (rolling on noble)";
}

#endif
