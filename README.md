# rclcpp_async

[![CI](https://github.com/otamachan/rclcpp_async/actions/workflows/ci.yml/badge.svg)](https://github.com/otamachan/rclcpp_async/actions/workflows/ci.yml)
[![Docs](https://img.shields.io/badge/docs-otamachan.github.io-blue)](https://otamachan.github.io/rclcpp_async/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue.svg)
![ROS 2 Rolling](https://img.shields.io/badge/ROS%202-Rolling-blue.svg)

A header-only C++20 coroutine library that brings `async/await` to ROS 2, inspired by [icey](https://github.com/iv461/icey).

**Documentation: <https://otamachan.github.io/rclcpp_async/>**

Write asynchronous ROS 2 code that reads like sequential code -- no callback nesting, no deadlocks, no `std::mutex`, no state machines -- all on a single-threaded executor.

```cpp
// Send two service requests in parallel with a 5s timeout
auto result = co_await ctx.wait_for(
  when_all(
    ctx.send_request<Srv1>(client1, req1),
    ctx.send_request<Srv2>(client2, req2)),
  5s);

// Both completed within 5s -- get the responses
if (result.ok()) {
  auto [resp1, resp2] = *result.value;
}
```

## Requirements

- ROS 2 Jazzy (or later)
- GCC 13+ with C++20 support
- `rclcpp`, `rclcpp_action`, `tf2_ros`

## Installation

Clone into your workspace and build:

```bash
cd ~/ros2_ws/src
git clone <repo-url> rclcpp_async
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rclcpp_async
```

## Quick Start

```cpp
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_async/rclcpp_async.hpp"

rclcpp_async::Task<void> run(rclcpp_async::CoContext & ctx)
{
  co_await ctx.sleep(std::chrono::seconds(1));
  RCLCPP_INFO(ctx.node().get_logger(), "Hello from coroutine!");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("hello");
  rclcpp_async::CoContext ctx(*node);

  auto task = ctx.create_task(run(ctx));

  rclcpp::spin(node);
  rclcpp::shutdown();
}
```

Every coroutine returns `Task<T>` and is launched with `ctx.create_task(...)`. The standard `rclcpp::spin()` drives execution -- no special executor needed.

See the [Guide](https://otamachan.github.io/rclcpp_async/guide/) for topics, services, actions, concurrency primitives, cancellation, and TF lookups.

## Benchmarks

See [rclcpp_async/test/benchmark/README.md](rclcpp_async/test/benchmark/README.md) for benchmark results comparing coroutine vs callback performance.

## License

Apache-2.0
