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

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

#include "bench_utils.hpp"
#include "rclcpp_async/rclcpp_async.hpp"

using namespace rclcpp_async;          // NOLINT(build/namespaces)
using namespace std::chrono_literals;  // NOLINT(build/namespaces)

static constexpr int N = 50;

static void run_bench(
  rclcpp::Node::SharedPtr & node, CoContext & ctx,
  rclcpp::executors::SingleThreadedExecutor & executor, std::chrono::nanoseconds duration)
{
  double expected_us = bench::to_us(duration);

  // --- std::this_thread::sleep_for ---
  std::vector<double> thread_samples;
  thread_samples.reserve(N);
  for (int i = 0; i < N; i++) {
    auto t0 = bench::Clock::now();
    std::this_thread::sleep_for(duration);
    thread_samples.push_back(bench::to_us(t0, bench::Clock::now()));
  }

  // --- rclcpp wall timer callback ---
  std::vector<double> callback_samples;
  callback_samples.reserve(N);
  for (int i = 0; i < N; i++) {
    bool done = false;
    bench::Clock::time_point t1;
    auto t0 = bench::Clock::now();
    auto timer = node->create_wall_timer(duration, [&done, &t1]() {
      t1 = bench::Clock::now();
      done = true;
    });
    while (!done) {
      executor.spin_some();
    }
    timer->cancel();
    timer.reset();
    callback_samples.push_back(bench::to_us(t0, t1));
  }

  // --- co_await ctx.sleep() ---
  std::vector<double> coro_samples;
  coro_samples.reserve(N);
  for (int i = 0; i < N; i++) {
    bench::Clock::time_point t0, t1;
    auto coro = [&]() -> Task<void> {
      t0 = bench::Clock::now();
      co_await ctx.sleep(duration);
      t1 = bench::Clock::now();
    };
    auto task = ctx.create_task(coro());
    while (!task.handle.done()) {
      executor.spin_some();
    }
    coro_samples.push_back(bench::to_us(t0, t1));
  }

  auto ts = bench::compute_stats(thread_samples);
  auto cs = bench::compute_stats(callback_samples);
  auto co = bench::compute_stats(coro_samples);

  std::cout << std::fixed << std::setprecision(1);
  std::cout << "  " << std::setw(10) << expected_us << " us" << "  | sleep_for: " << std::setw(8)
            << (ts.mean_us - expected_us) << "  | callback: " << std::setw(8)
            << (cs.mean_us - expected_us) << "  | coroutine: " << std::setw(8)
            << (co.mean_us - expected_us) << "  | coro-cb: " << std::setw(8)
            << ((co.mean_us - expected_us) - (cs.mean_us - expected_us)) << " us" << std::endl;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("bench_sleep_node");
  auto ctx = std::make_unique<CoContext>(*node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::cout << "\n=== Sleep Overhead Benchmark (N=" << N << ") ===\n";
  std::cout << "  " << std::setw(10) << "duration" << "  | " << std::setw(19)
            << "sleep_for overhead" << "  | " << std::setw(18) << "callback overhead" << "  | "
            << std::setw(19) << "coroutine overhead" << "  | " << std::setw(18) << "coro - callback"
            << std::endl;
  std::cout << std::string(100, '-') << std::endl;

  for (auto dur : {1ms, 5ms, 10ms, 50ms, 100ms}) {
    run_bench(node, *ctx, executor, dur);
  }

  std::cout << std::endl;

  ctx.reset();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
