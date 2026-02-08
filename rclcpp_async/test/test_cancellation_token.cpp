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

#include <atomic>
#include <thread>
#include <vector>

#include "rclcpp_async/cancellation_token.hpp"

using namespace rclcpp_async;  // NOLINT(build/namespaces)

TEST(CancellationToken, InitiallyNotCancelled)
{
  CancellationToken token;
  EXPECT_FALSE(token.is_cancelled());
}

TEST(CancellationToken, CancelSetsFlag)
{
  CancellationToken token;
  token.cancel();
  EXPECT_TRUE(token.is_cancelled());
}

TEST(CancellationToken, CallbackInvokedOnCancel)
{
  CancellationToken token;
  bool called = false;
  token.on_cancel([&] { called = true; });
  EXPECT_FALSE(called);
  token.cancel();
  EXPECT_TRUE(called);
}

TEST(CancellationToken, MultipleCallbacksAllInvoked)
{
  CancellationToken token;
  int count = 0;
  token.on_cancel([&] { count++; });
  token.on_cancel([&] { count++; });
  token.on_cancel([&] { count++; });
  token.cancel();
  EXPECT_EQ(count, 3);
}

TEST(CancellationToken, CallbackInvokedImmediatelyIfAlreadyCancelled)
{
  CancellationToken token;
  token.cancel();
  bool called = false;
  token.on_cancel([&] { called = true; });
  EXPECT_TRUE(called);
}

TEST(CancellationToken, DoubleCancelIsHarmless)
{
  CancellationToken token;
  int count = 0;
  token.on_cancel([&] { count++; });
  token.cancel();
  token.cancel();
  // Callback is moved out on first cancel, so second cancel does nothing extra
  EXPECT_EQ(count, 1);
}

TEST(CancellationToken, ThreadSafety)
{
  CancellationToken token;
  std::atomic<int> count{0};
  constexpr int N = 100;

  // Register callbacks from multiple threads
  std::vector<std::thread> threads;
  for (int i = 0; i < N; i++) {
    threads.emplace_back([&] { token.on_cancel([&] { count.fetch_add(1); }); });
  }
  for (auto & t : threads) {
    t.join();
  }

  token.cancel();
  EXPECT_EQ(count.load(), N);
}

TEST(CancellationToken, ConcurrentCancelAndRegister)
{
  // Ensure no crash when cancel and on_cancel race
  for (int trial = 0; trial < 100; trial++) {
    CancellationToken token;
    std::atomic<int> count{0};

    std::thread t1([&] { token.on_cancel([&] { count.fetch_add(1); }); });
    std::thread t2([&] { token.cancel(); });

    t1.join();
    t2.join();

    // Callback must have been called exactly once
    EXPECT_EQ(count.load(), 1);
  }
}
