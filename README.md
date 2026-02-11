# rclcpp_async

A header-only C++20 coroutine library that brings `async/await` to ROS 2, inspired by [icey](https://github.com/iv461/icey).

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
    auto msg = co_await stream->next();
    if (!msg.has_value()) {
      break;
    }
    RCLCPP_INFO(ctx.node()->get_logger(), "Heard: %s", (*msg)->data.c_str());
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

  auto resp = co_await ctx.send_request<SetBool>(client, req);
  RCLCPP_INFO(ctx.node()->get_logger(), "Response: %s", resp->message.c_str());
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

### Single-Goal Action Server

`create_single_goal_action_server<ActionT>(ctx, name, callback)` creates an action server that executes only one goal at a time. When a new goal arrives while another is executing, the previous goal is preempted via `CancelledException` and the new goal starts only after the previous goal has fully completed.

```cpp
auto server = create_single_goal_action_server<Fibonacci>(
  ctx, "fibonacci",
  [&ctx](GoalContext<Fibonacci> goal) -> Task<Fibonacci::Result> {
    Fibonacci::Feedback feedback;
    feedback.sequence = {0, 1};

    for (int i = 2; i < goal.goal().order; i++) {
      auto n = feedback.sequence.size();
      feedback.sequence.push_back(
        feedback.sequence[n - 1] + feedback.sequence[n - 2]);
      goal.publish_feedback(feedback);
      co_await ctx.sleep(std::chrono::milliseconds(500));
      // If a new goal arrives, CancelledException is thrown here
    }

    Fibonacci::Result result;
    result.sequence = feedback.sequence;
    co_return result;
  });
```

Preemption is automatic -- the user callback simply uses `co_await` as normal, and `CancelledException` is thrown at the current suspension point when a new goal preempts the current one. Synchronous cleanup can be done in a `catch` block:

```cpp
[&ctx](GoalContext<Fibonacci> goal) -> Task<Fibonacci::Result> {
  try {
    // ... main execution ...
  } catch (const CancelledException &) {
    stop_motor();  // Cleanup runs before the next goal starts
    throw;
  }
}
```

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
    auto feedback = co_await stream->next();
    if (!feedback.has_value()) {
      break;
    }
    auto & seq = (*feedback)->sequence;
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
  co_await event.wait();
  RCLCPP_INFO(ctx.node()->get_logger(), "Event received!");
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
  co_await mutex.lock();
  RCLCPP_INFO(ctx.node()->get_logger(), "%s: acquired lock", name.c_str());
  co_await ctx.sleep(std::chrono::seconds(1));
  mutex.unlock();
}
```

| Method | Description |
|---|---|
| `mutex.lock()` | `co_await` -- suspends until the lock is acquired |
| `mutex.unlock()` | Release the lock, resuming the next waiter |
| `mutex.is_locked()` | Check the current state |

### TF Lookup (TfBuffer)

`TfBuffer` subscribes to `/tf` and `/tf_static` on a dedicated listener thread and provides `co_await`-able transform lookups.

```cpp
Task<void> run(CoContext & ctx)
{
  TfBuffer tf(ctx);

  // Sync lookup (latest) -- returns nullopt if not yet available
  auto opt = tf.lookup_transform("map", "base_link");
  if (opt) {
    RCLCPP_INFO(ctx.node()->get_logger(), "x=%.2f", opt->transform.translation.x);
  }

  // Async lookup -- suspends until the transform becomes available
  auto t = co_await tf.lookup_transform("map", "base_link", rclcpp::Time(0));

  // With timeout
  auto result = co_await ctx.wait_for(
    tf.lookup_transform("map", "odom", rclcpp::Time(0)), 5s);

  // Parallel lookups
  auto [t1, t2] = co_await when_all(
    tf.lookup_transform("map", "odom", rclcpp::Time(0)),
    tf.lookup_transform("odom", "base_link", rclcpp::Time(0)));
}
```

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
    auto val = co_await ch.next();
    if (!val.has_value()) {
      break;                       // Channel closed
    }
    RCLCPP_INFO(ctx.node()->get_logger(), "Received: %d", *val);
  }
});
```

### when_all

`when_all` concurrently awaits multiple tasks and returns all results as a `std::tuple`. It accepts `Task<T>` objects as well as arbitrary awaitables (anything with `await_ready`/`await_suspend`/`await_resume`), which are automatically wrapped into tasks.

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

  // Awaitables can be passed directly -- no need to wrap in a Task
  auto req = std::make_shared<SetBool::Request>();
  auto [resp, _] = co_await when_all(
    ctx.send_request<SetBool>(client, req),
    ctx.sleep(std::chrono::seconds(1)));
}
```

`void` tasks return `std::monostate` in the tuple. Cancellation of the parent task propagates to all child tasks.

### when_any

`when_any` races multiple tasks concurrently and returns the result of whichever finishes first, wrapped in a `std::variant`. The remaining tasks are cancelled automatically. Like `when_all`, it accepts both `Task<T>` objects and arbitrary awaitables.

```cpp
Task<int> fast(CoContext & ctx) {
  co_await ctx.sleep(std::chrono::seconds(1));
  co_return 42;
}

Task<std::string> slow(CoContext & ctx) {
  co_await ctx.sleep(std::chrono::seconds(10));
  co_return std::string("too slow");
}

Task<void> run(CoContext & ctx)
{
  auto result = co_await when_any(
    ctx.create_task(fast(ctx)),
    ctx.create_task(slow(ctx)));
  // result is std::variant<int, std::string>

  if (result.index() == 0) {
    int value = std::get<0>(result);  // 42
  }
  // slow() is cancelled automatically
}
```

`void` tasks produce `std::monostate` in the variant. Cancellation of the parent task propagates to all child tasks.

### wait_for / poll_until

`wait_for` races any task or awaitable against a timeout, returning `Result<T>`. `poll_until` polls a predicate at a fixed interval with a built-in timeout.

- `ctx.wait_for(task, timeout)` races a task against a timeout using `when_any`, returning `Result<T>`.
- `ctx.wait_for(awaitable, timeout)` also accepts any awaitable directly (e.g., `sleep`, `send_request`), wrapping it in a `Task` automatically.
- `ctx.poll_until(pred, interval, timeout)` returns `Result<void>` -- polls `pred()` every `interval`, returning `Ok` when true or `Timeout` after `timeout`.

```cpp
Task<void> run(CoContext & ctx)
{
  auto client = ctx.node()->create_client<SetBool>("set_bool");

  // Race an awaitable against a timeout
  auto req = std::make_shared<SetBool::Request>();
  req->data = true;
  auto resp = co_await ctx.wait_for(ctx.send_request<SetBool>(client, req), 5s);
  if (resp.ok()) {
    RCLCPP_INFO(ctx.node()->get_logger(), "Response: %s", resp.value->message.c_str());
  }
}
```

`wait_for_service` and `wait_for_action` are built on top of `poll_until`. Cancellation propagates automatically through the entire chain.

## Result Type

Operations that can timeout or fail return `Result<T>`:

```cpp
auto result = co_await ctx.wait_for_service(client, std::chrono::seconds(10));

if (result.ok()) {
  // Service is available
} else if (result.timeout()) {
  // Timed out
}
```

Most operations return their value directly (e.g., `send_request` returns `Response`, `sleep` returns `void`). `Result<T>` is only used when timeout or error is a normal outcome:

| Operation | Return type | Uses Result? |
|---|---|---|
| `sleep` | `void` | No |
| `send_request` | `Response` | No |
| `stream->next()` | `optional<T>` | No |
| `event.wait()` | `void` | No |
| `mutex.lock()` | `void` | No |
| `tf.lookup_transform` | `TransformStamped` | No |
| `poll_until` | `Result<void>` | Yes (timeout) |
| `wait_for` | `Result<T>` | Yes (timeout) |
| `wait_for_service` | `Result<void>` | Yes (timeout) |
| `wait_for_action` | `Result<void>` | Yes (timeout) |
| `send_goal` | `Result<GoalStream>` | Yes (rejected) |

## Cancellation

Calling `task.cancel()` cancels the task by throwing `CancelledException` at the current `co_await` suspension point. The exception propagates automatically through the coroutine chain -- no manual checking needed.

```cpp
auto task = ctx.create_task(run(ctx));

// Later...
task.cancel();  // Throws CancelledException at the current co_await
```

To handle cancellation explicitly, catch `CancelledException`:

```cpp
Task<void> run(CoContext & ctx)
{
  try {
    co_await ctx.sleep(std::chrono::seconds(10));
  } catch (const CancelledException &) {
    RCLCPP_INFO(ctx.node()->get_logger(), "Cancelled!");
  }
}
```

## No Deadlocks on Single-Threaded Executors

A key advantage of coroutine-based I/O: nested service calls work without deadlocks, even on a single-threaded executor. For example, a service handler can `co_await` another service call, which can itself call yet another service -- all on the same thread.

See [`nested_demo.cpp`](rclcpp_async_example/src/nested_demo.cpp) for a full demonstration.

## API Reference

### CoContext

| Method | Returns | Description |
|---|---|---|
| `create_task(task)` | `Task<T>` | Start a coroutine |
| `subscribe<MsgT>(topic, qos)` | `shared_ptr<TopicStream<MsgT>>` | Subscribe to a topic |
| `create_timer(period)` | `shared_ptr<TimerStream>` | Create a periodic timer stream |
| `create_service<SrvT>(name, cb)` | `shared_ptr<Service<SrvT>>` | Create a coroutine service server |
| `create_action_server<ActT>(name, cb)` | `shared_ptr<Server<ActT>>` | Create a coroutine action server |
| `create_single_goal_action_server<ActT>(ctx, name, cb)` | `shared_ptr<Server<ActT>>` | Single-goal action server with preemption |
| `wait_for_service(client, timeout)` | *awaitable* `Result<void>` | Wait for a service |
| `send_request<SrvT>(client, req)` | *awaitable* `Response` | Call a service |
| `wait_for_action(client, timeout)` | *awaitable* `Result<void>` | Wait for an action server |
| `send_goal<ActT>(client, goal)` | *awaitable* `Result<shared_ptr<GoalStream>>` | Send an action goal |
| `sleep(duration)` | *awaitable* `void` | Async sleep |
| `poll_until(pred, interval, timeout)` | `Task<Result<void>>` | Poll until predicate is true or timeout |
| `wait_for(awaitable, timeout)` | `Task<Result<T>>` | Race an awaitable against a timeout |
| `when_all(awaitables...)` | *awaitable* `tuple<Ts...>` | Await all tasks/awaitables concurrently |
| `when_any(awaitables...)` | *awaitable* `variant<Ts...>` | Race tasks/awaitables, return first result |
| `post(fn)` | `void` | Post a callback to the executor thread (thread-safe) |
| `node()` | `Node::SharedPtr` | Access the underlying node |

### TfBuffer

| Method | Returns | Description |
|---|---|---|
| `TfBuffer(ctx)` | -- | Construct with a `CoContext`; starts a listener thread |
| `lookup_transform(target, source)` | `optional<TransformStamped>` | Sync lookup of latest transform |
| `lookup_transform(target, source, time)` | *awaitable* `TransformStamped` | Async lookup; suspends until the transform is available |

## Benchmarks

See [rclcpp_async/test/benchmark/README.md](rclcpp_async/test/benchmark/README.md) for benchmark results comparing coroutine vs callback performance.

## License

Apache-2.0
