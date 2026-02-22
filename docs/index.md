# rclcpp_async

A header-only C++20 coroutine library that brings **async/await** to ROS 2.

Write asynchronous ROS 2 code that reads like sequential code --
no callback nesting, no deadlocks, no `std::mutex`, no state machines --
all on a single-threaded executor.

## Features

- **Subscribers** -- async streams with `co_await stream->next()`
- **Service client** -- `co_await ctx.send_request(...)` with no shared futures
- **Service server** -- coroutine handlers that can `co_await` internally
- **Action client** -- goal streaming with `co_await stream->next()` for feedback
- **Action server** -- coroutine handlers that can `co_await` internally
- **Timers** -- periodic async streams
- **Concurrency primitives** -- `when_all`, `when_any`, `Event`, `Mutex`, `Channel`
- **Cancellation** -- `task.cancel()` throws `CancelledException` at `co_await` points
- **Timeout** -- `ctx.wait_for(awaitable, timeout)` returns `Result<T>`
- **Plus**
    - **TF lookups** -- `co_await tf.lookup_transform(...)` via `TfBuffer`
    - **Single-goal action server** -- handles one goal at a time (like ROS 1 `SimpleActionServer`) with automatic preemption

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

Every coroutine returns `Task<T>` and is launched with `ctx.create_task(...)`.
The standard `rclcpp::spin()` drives execution -- no special executor needed.

## Learn More

- **[Getting Started](getting-started.md)** -- prerequisites, installation, and a walkthrough of the Quick Start example
- **[Guide](guide.md)** -- topics, services, actions, concurrency primitives, cancellation, and TF lookups
