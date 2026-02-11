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
#include <std_srvs/srv/set_bool.hpp>
#include <vector>

#include "bench_utils.hpp"
#include "rclcpp_async/rclcpp_async.hpp"

using namespace rclcpp_async;          // NOLINT(build/namespaces)
using namespace std::chrono_literals;  // NOLINT(build/namespaces)
using SetBool = std_srvs::srv::SetBool;

static constexpr int N = 100;
static constexpr int WARMUP = 10;

static Task<void> service_coro(
  CoContext & ctx, rclcpp::Client<SetBool>::SharedPtr client, std::vector<double> & samples)
{
  for (int i = 0; i < WARMUP + N; i++) {
    auto req = std::make_shared<SetBool::Request>();
    req->data = true;
    auto t0 = bench::Clock::now();
    co_await ctx.send_request<SetBool>(client, req);
    auto t1 = bench::Clock::now();
    if (i >= WARMUP) {
      samples.push_back(bench::to_us(t0, t1));
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("bench_service_node");
  auto ctx = std::make_unique<CoContext>(*node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto server = node->create_service<SetBool>(
    "bench_service",
    [](const std::shared_ptr<SetBool::Request> req, std::shared_ptr<SetBool::Response> resp) {
      resp->success = req->data;
    });

  auto client = node->create_client<SetBool>("bench_service");

  while (!client->service_is_ready()) {
    executor.spin_some();
  }

  std::cout << "\n=== Service Call Latency Benchmark (N=" << N << ") ===" << std::endl;
  std::cout << std::fixed << std::setprecision(1);

  // --- callback client ---
  {
    std::vector<double> samples;
    samples.reserve(N);

    for (int i = 0; i < WARMUP + N; i++) {
      auto req = std::make_shared<SetBool::Request>();
      req->data = true;
      bool done = false;
      bench::Clock::time_point t1;
      auto t0 = bench::Clock::now();
      client->async_send_request(req, [&](rclcpp::Client<SetBool>::SharedFuture) {
        t1 = bench::Clock::now();
        done = true;
      });
      while (!done) {
        executor.spin_some();
      }
      if (i >= WARMUP) {
        samples.push_back(bench::to_us(t0, t1));
      }
    }
    bench::print_stats("callback", bench::compute_stats(samples));
  }

  // --- coroutine send_request (single task) ---
  {
    std::vector<double> samples;
    samples.reserve(N);

    auto task = ctx->create_task(service_coro(*ctx, client, samples));
    while (!task.handle.done()) {
      executor.spin_some();
    }
    bench::print_stats("coroutine", bench::compute_stats(samples));
  }

  std::cout << std::endl;

  ctx.reset();
  server.reset();
  client.reset();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
