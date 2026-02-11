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
#include <vector>

#include "bench_utils.hpp"
#include "rclcpp_async/rclcpp_async.hpp"

using namespace rclcpp_async;          // NOLINT(build/namespaces)
using namespace std::chrono_literals;  // NOLINT(build/namespaces)

// Free coroutine functions — arguments are copied into the coroutine frame,
// avoiding the lambda-coroutine dangling-capture problem.
static Task<void> timer_task(std::shared_ptr<TimerStream> timer, int repeats, int * done_count)
{
  for (int r = 0; r < repeats; r++) {
    co_await timer->next();
  }
  (*done_count)++;
}

static Task<void> sleep_task(
  CoContext & ctx, std::chrono::nanoseconds duration, int repeats, int * done_count)
{
  for (int r = 0; r < repeats; r++) {
    co_await ctx.sleep(duration);
  }
  (*done_count)++;
}

static void run_bench(
  rclcpp::Node::SharedPtr & node, CoContext & ctx,
  rclcpp::executors::SingleThreadedExecutor & executor, int num_tasks, int repeats,
  std::chrono::nanoseconds duration)
{
  double sleep_us = bench::to_us(duration);

  // --- callback: N periodic timers, each fires `repeats` times ---
  {
    int done_count = 0;
    std::vector<int> fire_counts(num_tasks, 0);
    std::vector<rclcpp::TimerBase::SharedPtr> timers;
    timers.reserve(num_tasks);

    auto wall_t0 = bench::Clock::now();
    for (int i = 0; i < num_tasks; i++) {
      auto timer = node->create_wall_timer(duration, [i, repeats, &done_count, &fire_counts]() {
        fire_counts[i]++;
        if (fire_counts[i] == repeats) {
          done_count++;
        }
      });
      timers.push_back(timer);
    }
    while (done_count < num_tasks) {
      executor.spin_some();
    }
    double wall_us = bench::to_us(wall_t0, bench::Clock::now());
    for (auto & t : timers) {
      t->cancel();
    }
    timers.clear();

    std::cout << "  callback   " << "  tasks=" << std::setw(5) << num_tasks
              << "  repeats=" << repeats << "  sleep=" << std::setw(8) << sleep_us << " us"
              << "  wall=" << std::setw(10) << wall_us << " us" << std::endl;
  }

  // --- coroutine (sleep): N tasks, each co_await sleep() `repeats` times ---
  {
    int done_count = 0;

    auto wall_t0 = bench::Clock::now();
    std::vector<Task<void>> tasks;
    tasks.reserve(num_tasks);
    for (int i = 0; i < num_tasks; i++) {
      tasks.push_back(ctx.create_task(sleep_task(ctx, duration, repeats, &done_count)));
    }
    while (done_count < num_tasks) {
      executor.spin_some();
    }
    double wall_us = bench::to_us(wall_t0, bench::Clock::now());

    std::cout << "  coro/sleep " << "  tasks=" << std::setw(5) << num_tasks
              << "  repeats=" << repeats << "  sleep=" << std::setw(8) << sleep_us << " us"
              << "  wall=" << std::setw(10) << wall_us << " us" << std::endl;
  }

  // --- coroutine (TimerStream): N tasks, each co_await timer->next() `repeats`
  // times ---
  if (repeats > 1) {
    int done_count = 0;

    auto wall_t0 = bench::Clock::now();
    std::vector<Task<void>> tasks;
    std::vector<std::shared_ptr<TimerStream>> timers;
    tasks.reserve(num_tasks);
    timers.reserve(num_tasks);
    for (int i = 0; i < num_tasks; i++) {
      auto timer = ctx.create_timer(duration);
      timers.push_back(timer);
      tasks.push_back(ctx.create_task(timer_task(timer, repeats, &done_count)));
    }
    while (done_count < num_tasks) {
      executor.spin_some();
    }
    double wall_us = bench::to_us(wall_t0, bench::Clock::now());

    std::cout << "  coro/timer " << "  tasks=" << std::setw(5) << num_tasks
              << "  repeats=" << repeats << "  sleep=" << std::setw(8) << sleep_us << " us"
              << "  wall=" << std::setw(10) << wall_us << " us" << std::endl;
  }
  std::cout << std::endl;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("bench_concurrent_node");
  auto ctx = std::make_unique<CoContext>(node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::cout << "\n=== Concurrent Tasks Benchmark ===" << std::endl;
  std::cout << std::fixed << std::setprecision(1);

  for (int repeats : {1, 3}) {
    for (auto dur : {1ms, 10ms, 100ms}) {
      for (int n : {100, 500, 1000, 5000}) {
        run_bench(node, *ctx, executor, n, repeats, dur);
      }
    }
  }

  ctx.reset();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
