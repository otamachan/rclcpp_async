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
#include <std_msgs/msg/int64.hpp>
#include <vector>

#include "bench_utils.hpp"
#include "rclcpp_async/rclcpp_async.hpp"

using namespace rclcpp_async;          // NOLINT(build/namespaces)
using namespace std::chrono_literals;  // NOLINT(build/namespaces)

static constexpr int N = 100;
static constexpr int WARMUP = 10;

static Task<void> pubsub_coro(
  CoContext & ctx, rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub,
  std::vector<double> & samples)
{
  auto stream = ctx.subscribe<std_msgs::msg::Int64>("bench_topic", rclcpp::QoS(10));

  for (int i = 0; i < WARMUP + N; i++) {
    std_msgs::msg::Int64 msg;
    msg.data = i;
    auto t0 = bench::Clock::now();
    pub->publish(msg);
    co_await stream->next();
    auto t1 = bench::Clock::now();
    if (i >= WARMUP) {
      samples.push_back(bench::to_us(t0, t1));
    }
  }
  stream->close();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("bench_pubsub_node");
  auto ctx = std::make_unique<CoContext>(node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto pub = node->create_publisher<std_msgs::msg::Int64>("bench_topic", rclcpp::QoS(10));

  std::cout << "\n=== Pub/Sub Latency Benchmark (N=" << N << ") ===" << std::endl;
  std::cout << std::fixed << std::setprecision(1);

  // --- callback subscription ---
  {
    std::vector<double> samples;
    samples.reserve(N);
    bench::Clock::time_point t0;
    bool received = false;

    auto sub = node->create_subscription<std_msgs::msg::Int64>(
      "bench_topic", rclcpp::QoS(10), [&](std_msgs::msg::Int64::SharedPtr) {
        auto t1 = bench::Clock::now();
        samples.push_back(bench::to_us(t0, t1));
        received = true;
      });

    for (int i = 0; i < WARMUP + N; i++) {
      std_msgs::msg::Int64 msg;
      msg.data = i;
      received = false;
      t0 = bench::Clock::now();
      pub->publish(msg);
      while (!received) {
        executor.spin_some();
      }
      if (i == WARMUP - 1) {
        samples.clear();
      }
    }
    sub.reset();
    bench::print_stats("callback", bench::compute_stats(samples));
  }

  // --- coroutine TopicStream (single task) ---
  {
    std::vector<double> samples;
    samples.reserve(N);

    auto task = ctx->create_task(pubsub_coro(*ctx, pub, samples));
    while (!task.handle.done()) {
      executor.spin_some();
    }
    bench::print_stats("coroutine", bench::compute_stats(samples));
  }

  std::cout << std::endl;

  ctx.reset();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
