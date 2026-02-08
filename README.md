# rclcpp_async

A header-only C++20 coroutine library that brings `async/await` to ROS 2.

Write asynchronous ROS 2 code that reads like sequential code -- no callback nesting, no deadlocks on single-threaded executors.

## Requirements

- ROS 2 Jazzy (or later)
- GCC 13+ with C++20 support
- `rclcpp`, `rclcpp_action`

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
  RCLCPP_INFO(ctx.node()->get_logger(), "Hello from coroutine!");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("hello");
  rclcpp_async::CoContext ctx(node);

  auto task = ctx.create_task(run(ctx));

  rclcpp::spin(node);
  rclcpp::shutdown();
}
```

Every coroutine returns `Task<T>` and is launched with `ctx.create_task(...)`. The standard `rclcpp::spin()` drives execution -- no special executor needed.

## Usage

### create_task

`create_task` registers and starts a coroutine. It accepts either a `Task<T>` directly or a callable that returns one.

```cpp
// From a coroutine function
Task<void> my_coroutine(CoContext & ctx) { /* ... */ co_return; }
auto task = ctx.create_task(my_coroutine(ctx));

// From a lambda
auto task = ctx.create_task([&ctx]() -> Task<void> {
  co_await ctx.sleep(std::chrono::seconds(1));
});
```

You can launch multiple tasks concurrently -- they all run cooperatively on the single-threaded executor:

```cpp
auto task_a = ctx.create_task(do_work_a(ctx));
auto task_b = ctx.create_task(do_work_b(ctx));
// Both run concurrently via co_await yield points
```

### Subscriber (TopicStream)

`ctx.subscribe<MsgT>(topic, qos)` returns a `TopicStream` -- an async stream that yields messages via `co_await stream->next()`.

```cpp
#include <std_msgs/msg/string.hpp>

Task<void> listen(CoContext & ctx, std::string topic)
{
  auto stream = ctx.subscribe<std_msgs::msg::String>(topic, 10);

  while (true) {
    auto r = co_await stream->next();
    if (!r.ok() || !r.value->has_value()) {
      break;
    }
    auto & msg = *r.value.value();
    RCLCPP_INFO(ctx.node()->get_logger(), "Heard: %s", msg->data.c_str());
  }
}
```

You can subscribe to multiple topics in parallel by launching separate tasks:

```cpp
auto task_a = ctx.create_task(listen(ctx, "topic_a"));
auto task_b = ctx.create_task(listen(ctx, "topic_b"));
```

### Service Client

Call a service with `co_await` -- no shared futures, no spinning while waiting.

```cpp
#include <std_srvs/srv/set_bool.hpp>

using SetBool = std_srvs::srv::SetBool;

Task<void> call_service(CoContext & ctx)
{
  auto client = ctx.node()->create_client<SetBool>("set_bool");

  // Wait for the service to become available
  auto wait_result = co_await ctx.wait_for_service(client, std::chrono::seconds(10));
  if (!wait_result.ok()) {
    RCLCPP_ERROR(ctx.node()->get_logger(), "Service not available");
    co_return;
  }

  // Send request
  auto req = std::make_shared<SetBool::Request>();
  req->data = true;

  auto result = co_await ctx.send_request<SetBool>(client, req);
  if (result.ok()) {
    auto resp = result.value.value();
    RCLCPP_INFO(ctx.node()->get_logger(), "Response: %s", resp->message.c_str());
  }
}
```

### Timer (TimerStream)

`ctx.create_timer(period)` returns a `TimerStream` that ticks at the given interval.

```cpp
Task<void> tick(CoContext & ctx)
{
  auto timer = ctx.create_timer(std::chrono::seconds(1));
  int count = 0;

  while (true) {
    co_await timer->next();
    RCLCPP_INFO(ctx.node()->get_logger(), "Tick %d", count++);
  }
}
```

### Service Server

`ctx.create_service<ServiceT>(name, callback)` creates a service whose handler is a coroutine. The handler can use `co_await` internally (e.g., to call other services) without blocking the executor.

```cpp
auto service = ctx.create_service<SetBool>(
  "set_bool",
  [&ctx](SetBool::Request::SharedPtr req) -> Task<SetBool::Response> {
    // You can co_await inside the handler
    co_await ctx.sleep(std::chrono::milliseconds(100));

    SetBool::Response resp;
    resp.success = req->data;
    resp.message = req->data ? "enabled" : "disabled";
    co_return resp;
  });
```

### Action Server

`ctx.create_action_server<ActionT>(name, callback)` creates an action server with a coroutine handler. Use `GoalContext` to check for cancellation and publish feedback.

```cpp
#include <example_interfaces/action/fibonacci.hpp>

using Fibonacci = example_interfaces::action::Fibonacci;

auto action_server = ctx.create_action_server<Fibonacci>(
  "fibonacci",
  [&ctx](rclcpp_async::GoalContext<Fibonacci> goal) -> Task<Fibonacci::Result> {
    Fibonacci::Feedback feedback;
    feedback.sequence = {0, 1};

    for (int i = 2; i < goal.goal().order; i++) {
      if (goal.is_canceling()) {
        break;
      }
      auto n = feedback.sequence.size();
      feedback.sequence.push_back(
        feedback.sequence[n - 1] + feedback.sequence[n - 2]);
      goal.publish_feedback(feedback);
      co_await ctx.sleep(std::chrono::milliseconds(500));
    }

    Fibonacci::Result result;
    result.sequence = feedback.sequence;
    co_return result;
  });
```

`GoalContext<ActionT>` provides:

| Method | Description |
|---|---|
| `goal.goal()` | Access the goal request |
| `goal.is_canceling()` | Check if cancellation was requested |
| `goal.publish_feedback(fb)` | Send feedback to the client |
| `goal.abort()` | Abort the goal |

### Action Client

Send a goal and iterate over feedback with `GoalStream`:

```cpp
Task<void> send_action(CoContext & ctx)
{
  auto client = rclcpp_action::create_client<Fibonacci>(ctx.node(), "fibonacci");

  auto wait_result = co_await ctx.wait_for_action(client, std::chrono::seconds(10));
  if (!wait_result.ok()) { co_return; }

  Fibonacci::Goal goal;
  goal.order = 8;

  auto goal_result = co_await ctx.send_goal<Fibonacci>(client, goal);
  if (!goal_result.ok()) { co_return; }

  auto stream = *goal_result.value;

  // Iterate over feedback
  while (true) {
    auto r = co_await stream->next();
    if (!r.ok() || !r.value->has_value()) {
      break;
    }
    auto & seq = (*r.value.value())->sequence;
    RCLCPP_INFO(ctx.node()->get_logger(), "Feedback: last=%d", seq.back());
  }

  // Get final result
  auto result = stream->result();
  auto & seq = result.result->sequence;
  RCLCPP_INFO(ctx.node()->get_logger(), "Result: last=%d", seq.back());
}
```

### Event

`Event` is a coroutine-aware signal for synchronization between tasks.

```cpp
rclcpp_async::Event event(ctx);

// Task 1: wait for the event
auto waiter = ctx.create_task([&]() -> Task<void> {
  auto r = co_await event.wait();
  if (r.ok()) {
    RCLCPP_INFO(ctx.node()->get_logger(), "Event received!");
  }
});

// Task 2: signal the event
auto signaler = ctx.create_task([&]() -> Task<void> {
  co_await ctx.sleep(std::chrono::seconds(2));
  event.set();  // Wakes up all waiting coroutines
});
```

| Method | Description |
|---|---|
| `event.set()` | Signal the event, resuming all waiters |
| `event.clear()` | Reset the event so future `wait()` calls will suspend |
| `event.wait()` | `co_await` -- suspends until the event is set |
| `event.is_set()` | Check the current state |

### Mutex

`Mutex` provides mutual exclusion for coroutines. Unlike `std::mutex`, it suspends the coroutine instead of blocking the thread.

```cpp
rclcpp_async::Mutex mutex(ctx);

Task<void> critical_section(CoContext & ctx, Mutex & mutex, const std::string & name)
{
  auto r = co_await mutex.lock();
  if (r.ok()) {
    RCLCPP_INFO(ctx.node()->get_logger(), "%s: acquired lock", name.c_str());
    co_await ctx.sleep(std::chrono::seconds(1));
    mutex.unlock();
  }
}
```

| Method | Description |
|---|---|
| `mutex.lock()` | `co_await` -- suspends until the lock is acquired |
| `mutex.unlock()` | Release the lock, resuming the next waiter |
| `mutex.is_locked()` | Check the current state |

### Channel

`Channel<T>` is a thread-safe MPSC (multi-producer, single-consumer) channel for sending values from worker threads to coroutines.

```cpp
rclcpp_async::Channel<int> ch(ctx);

// Worker thread: send values
std::thread worker([&ch]() {
  for (int i = 0; i < 10; i++) {
    ch.send(i);                    // Thread-safe
  }
  ch.close();                      // Signal end of stream
});

// Coroutine: receive values
auto task = ctx.create_task([&]() -> Task<void> {
  while (true) {
    auto r = co_await ch.next();
    if (!r.ok() || !r.value->has_value()) {
      break;                       // Channel closed or cancelled
    }
    int val = *r.value.value();
    RCLCPP_INFO(ctx.node()->get_logger(), "Received: %d", val);
  }
});
```

### when_all

`when_all` concurrently awaits multiple tasks and returns all results as a `std::tuple`.

```cpp
Task<int> fetch_a(CoContext & ctx) {
  co_await ctx.sleep(std::chrono::seconds(1));
  co_return 42;
}

Task<std::string> fetch_b(CoContext & ctx) {
  co_await ctx.sleep(std::chrono::seconds(2));
  co_return std::string("hello");
}

Task<void> run(CoContext & ctx)
{
  auto [a, b] = co_await when_all(
    ctx.create_task(fetch_a(ctx)),
    ctx.create_task(fetch_b(ctx)));
  // a == 42, b == "hello"
  // Total time ~2s (parallel), not 3s (sequential)
}
```

`void` tasks return `std::monostate` in the tuple. Cancellation of the parent task propagates to all child tasks.

## Result Type

All `co_await` operations return `Result<T>`, which carries the outcome status:

```cpp
auto result = co_await ctx.sleep(std::chrono::seconds(1));

if (result.ok()) {
  // Success
} else if (result.timeout()) {
  // Timed out (for wait_for_service, wait_for_action)
} else if (result.cancelled()) {
  // Task was cancelled
}
```

For operations that return a value (e.g., `send_request`), access it via `result.value.value()`.

## Cancellation

Calling `task.cancel()` cancels the task and automatically propagates cancellation to any `co_await` operation the task is currently suspended on.

```cpp
auto task = ctx.create_task(run(ctx));

// Later...
task.cancel();  // The running co_await will return Result with cancelled() == true
```

## No Deadlocks on Single-Threaded Executors

A key advantage of coroutine-based I/O: nested service calls work without deadlocks, even on a single-threaded executor. For example, a service handler can `co_await` another service call, which can itself call yet another service -- all on the same thread.

See [`example/nested_demo.cpp`](rclcpp_async/example/nested_demo.cpp) for a full demonstration.

## API Reference

### CoContext

| Method | Returns | Description |
|---|---|---|
| `create_task(task)` | `Task<T>` | Start a coroutine |
| `subscribe<MsgT>(topic, qos)` | `shared_ptr<TopicStream<MsgT>>` | Subscribe to a topic |
| `create_timer(period)` | `shared_ptr<TimerStream>` | Create a periodic timer stream |
| `create_service<SrvT>(name, cb)` | `shared_ptr<Service<SrvT>>` | Create a coroutine service server |
| `create_action_server<ActT>(name, cb)` | `shared_ptr<Server<ActT>>` | Create a coroutine action server |
| `wait_for_service(client, timeout)` | *awaitable* `Result<void>` | Wait for a service |
| `send_request<SrvT>(client, req)` | *awaitable* `Result<Response>` | Call a service |
| `wait_for_action(client, timeout)` | *awaitable* `Result<void>` | Wait for an action server |
| `send_goal<ActT>(client, goal)` | *awaitable* `Result<shared_ptr<GoalStream>>` | Send an action goal |
| `sleep(duration)` | *awaitable* `Result<void>` | Async sleep |
| `post(fn)` | `void` | Post a callback to the executor thread (thread-safe) |
| `node()` | `Node::SharedPtr` | Access the underlying node |

## License

Apache-2.0
