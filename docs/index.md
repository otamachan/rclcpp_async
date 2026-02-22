# rclcpp_async

A header-only C++20 coroutine library that brings **async/await** to ROS 2.

Write asynchronous ROS 2 code that reads like sequential code --
no callback nesting, no deadlocks, no `std::mutex`, no state machines --
all on a single-threaded executor.

## Features

- **Subscribers** -- async streams with `co_await stream->next()`
- **Service client/server** -- `co_await ctx.send_request(...)` with coroutine handlers
- **Action client/server** -- goal streaming, single-goal preemption, `shield()`
- **Timers** -- periodic async streams
- **TF lookups** -- `co_await tf.lookup_transform(...)`
- **Concurrency primitives** -- `when_all`, `when_any`, `Event`, `Mutex`, `Channel`
- **Cancellation** -- `task.cancel()` throws `CancelledException` at `co_await` points
- **Timeout** -- `ctx.wait_for(awaitable, timeout)` returns `Result<T>`

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

