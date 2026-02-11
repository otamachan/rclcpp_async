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
#include <string>
#include <vector>

#include "rclcpp_async/rclcpp_async.hpp"

using namespace rclcpp_async;          // NOLINT(build/namespaces)
using namespace std::chrono_literals;  // NOLINT(build/namespaces)

class MutexTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_mutex_node");
    ctx_ = std::make_unique<CoContext>(*node_);
    executor_.add_node(node_);
  }

  void TearDown() override
  {
    ctx_.reset();
    node_.reset();
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(1ms);
    }
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
};

TEST_F(MutexTest, UncontestedLockUnlock)
{
  Mutex mutex(*ctx_);

  bool reached = false;
  auto coro = [&]() -> Task<void> {
    co_await mutex.lock();
    reached = true;
    mutex.unlock();
  };
  auto task = ctx_->create_task(coro());
  spin_for(100ms);

  EXPECT_TRUE(task.handle.done());
  EXPECT_TRUE(reached);
  EXPECT_FALSE(mutex.is_locked());
}

TEST_F(MutexTest, MutualExclusion)
{
  Mutex mutex(*ctx_);
  std::vector<std::string> log;

  auto worker_a = [&]() -> Task<void> {
    co_await mutex.lock();
    log.push_back("A:enter");
    co_await ctx_->sleep(50ms);
    log.push_back("A:exit");
    mutex.unlock();
  };

  auto worker_b = [&]() -> Task<void> {
    co_await ctx_->sleep(100ms);
    co_await mutex.lock();
    log.push_back("B:enter");
    co_await ctx_->sleep(50ms);
    log.push_back("B:exit");
    mutex.unlock();
  };

  auto task_a = ctx_->create_task(worker_a());
  auto task_b = ctx_->create_task(worker_b());

  spin_for(500ms);

  ASSERT_TRUE(task_a.handle.done());
  ASSERT_TRUE(task_b.handle.done());

  // A acquires first (B sleeps 100ms before locking)
  ASSERT_EQ(log.size(), 4u);
  EXPECT_EQ(log[0], "A:enter");
  EXPECT_EQ(log[1], "A:exit");
  EXPECT_EQ(log[2], "B:enter");
  EXPECT_EQ(log[3], "B:exit");
  EXPECT_FALSE(mutex.is_locked());
}

TEST_F(MutexTest, ThreeWayContention)
{
  Mutex mutex(*ctx_);
  std::vector<std::string> log;

  auto make_worker = [&](std::string name) -> Task<void> {
    co_await mutex.lock();
    log.push_back(name + ":enter");
    co_await ctx_->sleep(10ms);
    log.push_back(name + ":exit");
    mutex.unlock();
  };

  auto task_a = ctx_->create_task(make_worker("A"));
  auto task_b = ctx_->create_task(make_worker("B"));
  auto task_c = ctx_->create_task(make_worker("C"));

  spin_for(500ms);

  ASSERT_TRUE(task_a.handle.done());
  ASSERT_TRUE(task_b.handle.done());
  ASSERT_TRUE(task_c.handle.done());

  // All three ran sequentially (no interleaving)
  ASSERT_EQ(log.size(), 6u);
  for (int i = 0; i < 3; i++) {
    auto enter = log[i * 2];
    auto exit = log[i * 2 + 1];
    // Same worker for enter/exit pair
    EXPECT_EQ(enter.substr(0, enter.find(':')), exit.substr(0, exit.find(':')));
  }
  EXPECT_FALSE(mutex.is_locked());
}

TEST_F(MutexTest, CancelDuringLockWait)
{
  Mutex mutex(*ctx_);

  bool was_cancelled = false;

  // Worker A holds the lock
  auto worker_a = [&]() -> Task<void> {
    co_await mutex.lock();
    co_await ctx_->sleep(200ms);
    mutex.unlock();
  };

  // Worker B tries to lock — will be suspended waiting
  auto worker_b = [&]() -> Task<void> {
    co_await ctx_->sleep(10ms);  // ensure A locks first
    try {
      co_await mutex.lock();
    } catch (const CancelledException &) {
      was_cancelled = true;
    }
  };

  auto task_a = ctx_->create_task(worker_a());
  auto task_b = worker_b();
  auto running_b = ctx_->create_task(std::move(task_b));

  // Let B start waiting on the mutex
  for (int i = 0; i < 30; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(1ms);
  }

  running_b.cancel();
  spin_for(500ms);

  ASSERT_TRUE(running_b.handle.done());
  EXPECT_TRUE(was_cancelled);
  ASSERT_TRUE(task_a.handle.done());
  EXPECT_FALSE(mutex.is_locked());
}

TEST_F(MutexTest, CancelDoesNotTransferLock)
{
  Mutex mutex(*ctx_);

  std::vector<std::string> log;
  bool b_waiting = false;

  // Worker A holds the lock for a while
  auto worker_a = [&]() -> Task<void> {
    co_await mutex.lock();
    log.push_back("A:enter");
    // Hold the lock until B is confirmed waiting, then a bit more
    while (!b_waiting) {
      co_await ctx_->sleep(5ms);
    }
    co_await ctx_->sleep(50ms);
    log.push_back("A:exit");
    mutex.unlock();
  };

  // Worker B tries to lock — will be added to wait queue, then cancelled
  auto worker_b = [&]() -> Task<void> {
    // Wait for A to hold the lock
    co_await ctx_->sleep(20ms);
    b_waiting = true;
    try {
      co_await mutex.lock();
      log.push_back("B:enter");
      mutex.unlock();
    } catch (const CancelledException &) {
      log.push_back("B:cancelled");
    }
  };

  // Worker C tries to lock after B
  auto worker_c = [&]() -> Task<void> {
    co_await ctx_->sleep(30ms);
    co_await mutex.lock();
    log.push_back("C:enter");
    co_await ctx_->sleep(10ms);
    log.push_back("C:exit");
    mutex.unlock();
  };

  auto task_a = ctx_->create_task(worker_a());
  auto task_b = worker_b();
  auto running_b = ctx_->create_task(std::move(task_b));
  auto task_c = ctx_->create_task(worker_c());

  // Wait until B is confirmed to be waiting on the lock
  for (int i = 0; i < 100 && !b_waiting; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(b_waiting);

  // Give B one more spin to ensure it's in the wait queue
  for (int i = 0; i < 10; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(1ms);
  }

  running_b.cancel();
  spin_for(500ms);

  ASSERT_TRUE(task_a.handle.done());
  ASSERT_TRUE(running_b.handle.done());
  ASSERT_TRUE(task_c.handle.done());

  // B was cancelled (not given the lock), C got the lock after A
  bool b_was_cancelled = false;
  bool c_entered = false;
  for (const auto & entry : log) {
    if (entry == "B:cancelled") {
      b_was_cancelled = true;
    }
    if (entry == "C:enter") {
      c_entered = true;
    }
  }
  EXPECT_TRUE(b_was_cancelled);
  EXPECT_TRUE(c_entered);
  EXPECT_FALSE(mutex.is_locked());
}
