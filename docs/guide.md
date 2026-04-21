# Guide

## Basics

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

## Topics and Timers

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
    RCLCPP_INFO(ctx.node().get_logger(), "Heard: %s", (*msg)->data.c_str());
  }
}
```

You can subscribe to multiple topics in parallel by launching separate tasks:

```cpp
auto task_a = ctx.create_task(listen(ctx, "topic_a"));
auto task_b = ctx.create_task(listen(ctx, "topic_b"));
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
    RCLCPP_INFO(ctx.node().get_logger(), "Tick %d", count++);
  }
}
```

## Services

### Service Client

Call a service with `co_await` -- no shared futures, no spinning while waiting.

```cpp
#include <std_srvs/srv/set_bool.hpp>

using SetBool = std_srvs::srv::SetBool;

Task<void> call_service(CoContext & ctx)
{
  auto client = ctx.node().create_client<SetBool>("set_bool");

  // Wait for the service to become available
  auto wait_result = co_await ctx.wait_for_service(client, std::chrono::seconds(10));
  if (!wait_result.ok()) {
    RCLCPP_ERROR(ctx.node().get_logger(), "Service not available");
    co_return;
  }

  // Send request
  auto req = std::make_shared<SetBool::Request>();
  req->data = true;

  auto resp = co_await ctx.send_request<SetBool>(client, req);
  RCLCPP_INFO(ctx.node().get_logger(), "Response: %s", resp->message.c_str());
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

### Nested Service Calls

A key advantage of coroutine-based I/O: nested service calls work without deadlocks, even on a single-threaded executor. A service handler can `co_await` another service call, which can itself call yet another service -- all on the same thread.

See [`nested_demo.cpp`](https://github.com/otamachan/rclcpp_async/blob/main/rclcpp_async_example/src/nested_demo.cpp) for a full demonstration.

## Actions

### Action Client

Send a goal and iterate over feedback with `GoalStream`:

```cpp
#include <example_interfaces/action/fibonacci.hpp>

using Fibonacci = example_interfaces::action::Fibonacci;

Task<void> send_action(CoContext & ctx)
{
  auto client = rclcpp_action::create_client<Fibonacci>(&ctx.node(), "fibonacci");

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
    RCLCPP_INFO(ctx.node().get_logger(), "Feedback: last=%d", seq.back());
  }

  // Get final result
  auto result = stream->result();
  auto & seq = result.result->sequence;
  RCLCPP_INFO(ctx.node().get_logger(), "Result: last=%d", seq.back());
}
```

`GoalStream` provides:

| Method | Description |
|---|---|
| `stream->next()` | `co_await` -- yields the next feedback (`nullopt` when the goal completes) |
| `stream->result()` | Access the final `WrappedResult` after the stream ends |
| `stream->cancel_goal()` | `co_await` -- request server-side cancellation and receive the response |
| `stream->set_auto_cancel_on_stop(bool)` | Toggle auto-cancelling the goal when the awaiting task is cancelled (default: `true`) |

#### Feedback queue depth

The feedback queue is bounded -- when it overflows, the oldest feedback is dropped (the final completion event is always preserved). The default depth is 10; pass a third argument to override:

```cpp
auto goal_result = co_await ctx.send_goal<Fibonacci>(client, goal, /*max_depth=*/32);
```

#### Auto-cancel on task cancel

If the awaiting task is cancelled while `co_await stream->next()` is suspended, the in-flight goal is automatically cancelled server-side (fire-and-forget `async_cancel_goal`) so the server does not waste work producing a result no one is listening for. Call `stream->set_auto_cancel_on_stop(false)` to opt out -- for example, if you want the goal to keep running in the background after the client gives up.

To request cancellation explicitly and wait for the server's response, use `cancel_goal()`:

```cpp
auto resp = co_await stream->cancel_goal();
// resp->return_code indicates whether the server accepted the cancel request
```

### Action Server

`ctx.create_action_server<ActionT>(name, callback)` creates an action server with a coroutine handler. Use `GoalContext` to check for cancellation and publish feedback.

```cpp
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

`create_single_goal_action_server<ActionT>(ctx, name, callback)` creates an action server that handles one goal at a time (like ROS 1 `SimpleActionServer`). When a new goal arrives while another is executing, the previous goal is preempted via `CancelledException` and the new goal starts only after the previous goal has fully completed.

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

Both preemption (new goal arrives) and client cancel automatically throw `CancelledException` at the current `co_await` suspension point -- no need to poll `goal.is_canceling()`. Synchronous cleanup can be done in a `catch` block:

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

## Concurrency

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

### Event

`Event` is a coroutine-aware signal for synchronization between tasks.

```cpp
rclcpp_async::Event event(ctx);

// Task 1: wait for the event
auto waiter = ctx.create_task([&]() -> Task<void> {
  co_await event.wait();
  RCLCPP_INFO(ctx.node().get_logger(), "Event received!");
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
  RCLCPP_INFO(ctx.node().get_logger(), "%s: acquired lock", name.c_str());
  co_await ctx.sleep(std::chrono::seconds(1));
  mutex.unlock();
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
    auto val = co_await ch.next();
    if (!val.has_value()) {
      break;                       // Channel closed
    }
    RCLCPP_INFO(ctx.node().get_logger(), "Received: %d", *val);
  }
});
```

## Cancellation and Timeouts

### cancel

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
    RCLCPP_INFO(ctx.node().get_logger(), "Cancelled!");
  }
}
```

### shield

`shield(task)` prevents a parent's cancellation from propagating to the inner task. The shielded task runs to completion even if the parent is cancelled. After the shielded task finishes, `CancelledException` is thrown at the next unshielded `co_await`.

```cpp
Task<void> run(CoContext & ctx)
{
  // This task completes even if run() is cancelled during it
  co_await shield(save_critical_data(ctx));

  // CancelledException is thrown here if cancelled
  co_await ctx.sleep(std::chrono::seconds(1));
}
```

### wait_for / poll_until

`wait_for` races any task or awaitable against a timeout, returning `Result<T>`. `poll_until` polls a predicate at a fixed interval with a built-in timeout.

- `ctx.wait_for(task, timeout)` races a task against a timeout using `when_any`, returning `Result<T>`.
- `ctx.wait_for(awaitable, timeout)` also accepts any awaitable directly (e.g., `sleep`, `send_request`), wrapping it in a `Task` automatically.
- `ctx.poll_until(pred, interval, timeout)` returns `Result<void>` -- polls `pred()` every `interval`, returning `Ok` when true or `Timeout` after `timeout`.

```cpp
Task<void> run(CoContext & ctx)
{
  auto client = ctx.node().create_client<SetBool>("set_bool");

  // Race an awaitable against a timeout
  auto req = std::make_shared<SetBool::Request>();
  req->data = true;
  auto resp = co_await ctx.wait_for(ctx.send_request<SetBool>(client, req), 5s);
  if (resp.ok()) {
    RCLCPP_INFO(ctx.node().get_logger(), "Response: %s", resp.value->message.c_str());
  }
}
```

`wait_for_service` and `wait_for_action` are built on top of `poll_until`. Cancellation propagates automatically through the entire chain.

### Result

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

## TF Lookups

`TfBuffer` subscribes to `/tf` and `/tf_static` on a dedicated listener thread and provides `co_await`-able transform lookups.

```cpp
Task<void> run(CoContext & ctx)
{
  TfBuffer tf(ctx);

  // Sync lookup (latest) -- returns nullopt if not yet available
  auto opt = tf.lookup_transform("map", "base_link");
  if (opt) {
    RCLCPP_INFO(ctx.node().get_logger(), "x=%.2f", opt->transform.translation.x);
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
