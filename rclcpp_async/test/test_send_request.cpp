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
#include <std_srvs/srv/set_bool.hpp>

#include "rclcpp_async/rclcpp_async.hpp"

using namespace rclcpp_async;          // NOLINT(build/namespaces)
using namespace std::chrono_literals;  // NOLINT(build/namespaces)
using SetBool = std_srvs::srv::SetBool;

class SendRequestTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_sendreq_node");
    ctx_ = std::make_unique<CoContext>(node_);
    executor_.add_node(node_);
    client_ = node_->create_client<SetBool>("test_sendreq_service");
    service_ = node_->create_service<SetBool>(
      "test_sendreq_service",
      [](const std::shared_ptr<SetBool::Request> req, std::shared_ptr<SetBool::Response> resp) {
        resp->success = req->data;
        resp->message = req->data ? "enabled" : "disabled";
      });
  }

  void TearDown() override
  {
    service_.reset();
    client_.reset();
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
  rclcpp::Client<SetBool>::SharedPtr client_;
  rclcpp::Service<SetBool>::SharedPtr service_;
};

TEST_F(SendRequestTest, SendAndReceive)
{
  // Wait for service discovery
  for (int i = 0; i < 50; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(10ms);
    if (client_->service_is_ready()) {
      break;
    }
  }
  ASSERT_TRUE(client_->service_is_ready());

  Result<SetBool::Response::SharedPtr> result;
  auto coro = [&]() -> Task<void> {
    auto req = std::make_shared<SetBool::Request>();
    req->data = true;
    result = co_await ctx_->send_request<SetBool>(client_, req);
  };
  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  ASSERT_TRUE(result.ok());
  EXPECT_TRUE(result.value.value()->success);
  EXPECT_EQ(result.value.value()->message, "enabled");
}

TEST_F(SendRequestTest, SendFalseValue)
{
  for (int i = 0; i < 50; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(10ms);
    if (client_->service_is_ready()) {
      break;
    }
  }
  ASSERT_TRUE(client_->service_is_ready());

  Result<SetBool::Response::SharedPtr> result;
  auto coro = [&]() -> Task<void> {
    auto req = std::make_shared<SetBool::Request>();
    req->data = false;
    result = co_await ctx_->send_request<SetBool>(client_, req);
  };
  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  ASSERT_TRUE(result.ok());
  EXPECT_FALSE(result.value.value()->success);
  EXPECT_EQ(result.value.value()->message, "disabled");
}

TEST_F(SendRequestTest, AlreadyCancelledIsImmediate)
{
  for (int i = 0; i < 50; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(10ms);
    if (client_->service_is_ready()) {
      break;
    }
  }

  Result<SetBool::Response::SharedPtr> result;
  auto coro = [&]() -> Task<void> {
    auto req = std::make_shared<SetBool::Request>();
    req->data = true;
    result = co_await ctx_->send_request<SetBool>(client_, req);
  };
  auto task = coro();
  task.cancel();
  auto running = ctx_->create_task(std::move(task));
  spin_until_done(running);

  ASSERT_TRUE(running.handle.done());
  EXPECT_TRUE(result.cancelled());
}

TEST_F(SendRequestTest, CancelDuringSendRequest)
{
  // Use a timer to cancel the task while send_request is suspended.
  // The service is not discovered yet, so the request will be pending.
  // Create a client for a non-existent service to ensure the request stays pending.
  auto orphan_client = node_->create_client<SetBool>("nonexistent_service");

  Result<SetBool::Response::SharedPtr> result;
  auto coro = [&]() -> Task<void> {
    auto req = std::make_shared<SetBool::Request>();
    req->data = true;
    result = co_await ctx_->send_request<SetBool>(orphan_client, req);
  };

  auto task = coro();
  auto running = ctx_->create_task(std::move(task));

  // Let the coroutine suspend on send_request
  for (int i = 0; i < 10; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(1ms);
  }

  running.cancel();
  spin_until_done(running);

  ASSERT_TRUE(running.handle.done());
  EXPECT_TRUE(result.cancelled());
}
