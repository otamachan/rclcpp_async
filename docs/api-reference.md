# API Reference

## CoContext

| Method | Returns | Description |
|---|---|---|
| `create_task(task)` | `Task<T>` | Start a coroutine |
| `subscribe<MsgT>(topic, qos)` | `shared_ptr<TopicStream<MsgT>>` | Subscribe to a topic |
| `create_timer(period)` | `shared_ptr<TimerStream>` | Create a periodic timer stream |
| `create_service<SrvT>(name, cb)` | `shared_ptr<Service<SrvT>>` | Create a coroutine service server |
| `create_action_server<ActT>(name, cb)` | `shared_ptr<Server<ActT>>` | Create a coroutine action server |
| `wait_for_service(client, timeout)` | *awaitable* `Result<void>` | Wait for a service |
| `send_request<SrvT>(client, req)` | *awaitable* `Response` | Call a service |
| `wait_for_action(client, timeout)` | *awaitable* `Result<void>` | Wait for an action server |
| `send_goal<ActT>(client, goal)` | *awaitable* `Result<shared_ptr<GoalStream>>` | Send an action goal |
| `sleep(duration)` | *awaitable* `void` | Async sleep |
| `poll_until(pred, interval, timeout)` | `Task<Result<void>>` | Poll until predicate is true or timeout |
| `wait_for(awaitable, timeout)` | `Task<Result<T>>` | Race an awaitable against a timeout |
| `post(fn)` | `void` | Post a callback to the executor thread (thread-safe) |
| `node()` | `Node&` | Access the underlying node |

## Free Functions

| Function | Returns | Description |
|---|---|---|
| `create_single_goal_action_server<ActT>(ctx, name, cb)` | `shared_ptr<Server<ActT>>` | Single-goal action server with preemption |
| `when_all(awaitables...)` | *awaitable* `tuple<Ts...>` | Await all tasks/awaitables concurrently |
| `when_any(awaitables...)` | *awaitable* `variant<Ts...>` | Race tasks/awaitables, return first result |
| `shield(task)` | *awaitable* `T` | Protect a task from parent cancellation |

## Task

| Method | Returns | Description |
|---|---|---|
| `operator bool()` | `bool` | `true` if the task holds a valid coroutine (not moved-from) |
| `done()` | `bool` | `true` if the coroutine has completed (`false` for null tasks) |
| `result()` | `T&` | Get the result of a completed `Task<T>` (rethrows if exception) |
| `cancel()` | `void` | Request cancellation via `CancelledException` |

## Result

| Method | Returns | Description |
|---|---|---|
| `ok()` | `bool` | `true` if the operation succeeded |
| `timeout()` | `bool` | `true` if the operation timed out |
| `value` | `std::optional<T>` | The result value (present when `ok()` is `true`) |

## GoalContext

| Method | Returns | Description |
|---|---|---|
| `goal()` | `const Goal&` | Access the goal request |
| `is_canceling()` | `bool` | Check if cancellation was requested |
| `publish_feedback(fb)` | `void` | Send feedback to the client |
| `abort()` | `void` | Abort the goal |

## Event

| Method | Returns | Description |
|---|---|---|
| `set()` | `void` | Signal the event, resuming all waiters |
| `clear()` | `void` | Reset the event so future `wait()` calls will suspend |
| `wait()` | *awaitable* `void` | Suspends until the event is set |
| `is_set()` | `bool` | Check the current state |

## Mutex

| Method | Returns | Description |
|---|---|---|
| `lock()` | *awaitable* `void` | Suspends until the lock is acquired |
| `unlock()` | `void` | Release the lock, resuming the next waiter |
| `is_locked()` | `bool` | Check the current state |

## Channel

| Method | Returns | Description |
|---|---|---|
| `send(value)` | `void` | Send a value (thread-safe) |
| `close()` | `void` | Signal end of stream |
| `next()` | *awaitable* `optional<T>` | Receive the next value (`nullopt` when closed) |

## TfBuffer

| Method | Returns | Description |
|---|---|---|
| `TfBuffer(ctx)` | -- | Construct with a `CoContext`; starts a listener thread |
| `lookup_transform(target, source)` | `optional<TransformStamped>` | Sync lookup of latest transform |
| `lookup_transform(target, source, time)` | *awaitable* `TransformStamped` | Async lookup; suspends until available |
