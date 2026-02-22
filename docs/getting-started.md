# Getting Started

## Prerequisites

- **ROS 2 Jazzy** (or later)
- **GCC 13+** with C++20 support
- ROS 2 packages: `rclcpp`, `rclcpp_action`, `tf2_ros`

## Installation

rclcpp_async is header-only. Clone it into your workspace and build:

```bash
cd ~/ros2_ws/src
git clone https://github.com/otamachan/rclcpp_async.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rclcpp_async
```

## Quick Start

Here is a minimal coroutine that sleeps for one second and prints a message:

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

### Walkthrough

1. **`Task<void> run(...)`** -- Every coroutine returns `Task<T>`. Use `co_await` inside to suspend without blocking the thread.

2. **`CoContext ctx(*node)`** -- The bridge between ROS 2 and coroutines. It uses the node's single-threaded executor to schedule coroutine resumptions via timers and callbacks.

3. **`ctx.create_task(run(ctx))`** -- Registers and starts the coroutine. Without this call, the coroutine object is created but never executed.

4. **`rclcpp::spin(node)`** -- The standard ROS 2 spin loop drives everything. Each `co_await` yields control back to the executor, which processes other callbacks and resumes coroutines when their results are ready.

## How It Works

rclcpp_async runs all coroutines cooperatively on a **single-threaded executor**:

```
rclcpp::spin(node)
  ├── timer fires       → resumes sleeping coroutine
  ├── message arrives   → resumes subscriber coroutine
  ├── service response  → resumes client coroutine
  └── ...
```

Every `co_await` is a **suspension point** -- the coroutine pauses and returns control to the executor. When the awaited event completes (timer fires, message arrives, etc.), the executor resumes the coroutine exactly where it left off.

This means:

- **No threads** -- no `std::mutex`, no data races
- **No deadlocks** -- nested service calls work on a single thread
- **No callbacks** -- sequential code with `co_await` instead of callback nesting
