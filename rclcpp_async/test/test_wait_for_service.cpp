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

class WaitForServiceTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_wfs_node");
    ctx_ = std::make_unique<CoContext>(node_);
    executor_.add_node(node_);
    client_ = node_->create_client<SetBool>("test_wfs_service");
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

  void create_service()
  {
    service_ = node_->create_service<SetBool>(
      "test_wfs_service",
      [](const std::shared_ptr<SetBool::Request>, std::shared_ptr<SetBool::Response> resp) {
        resp->success = true;
      });
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<CoContext> ctx_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<SetBool>::SharedPtr client_;
  rclcpp::Service<SetBool>::SharedPtr service_;
};

TEST_F(WaitForServiceTest, ServiceAlreadyReady)
{
  create_service();
  // Spin to discover the service
  for (int i = 0; i < 50; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(10ms);
    if (client_->service_is_ready()) {
      break;
    }
  }

  Result<void> result;
  auto coro = [&]() -> Task<void> { result = co_await ctx_->wait_for_service(client_, 1s); };
  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(result.ok());
}

TEST_F(WaitForServiceTest, ServiceBecomesReady)
{
  Result<void> result;
  auto coro = [&]() -> Task<void> { result = co_await ctx_->wait_for_service(client_, 5s); };
  auto task = ctx_->create_task(coro());

  // Spin a bit, then create the service
  for (int i = 0; i < 10; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  create_service();
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(result.ok());
}

TEST_F(WaitForServiceTest, Timeout)
{
  Result<void> result;
  auto coro = [&]() -> Task<void> { result = co_await ctx_->wait_for_service(client_, 200ms); };
  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(result.timeout());
}

TEST_F(WaitForServiceTest, Cancel)
{
  bool was_cancelled = false;
  auto coro = [&]() -> Task<void> {
    try {
      co_await ctx_->wait_for_service(client_, 10s);
    } catch (const CancelledException &) {
      was_cancelled = true;
    }
  };
  auto task = ctx_->create_task(coro());

  executor_.spin_some();
  std::this_thread::sleep_for(10ms);
  executor_.spin_some();

  task.cancel();
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(was_cancelled);
}
