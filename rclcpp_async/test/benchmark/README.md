# Benchmarks

## Build & Run

```bash
# Build (Release)
./run.sh bash -c "source /opt/ros/\$ROS_DISTRO/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Run individually
./run.sh bash -c "source /opt/ros/\$ROS_DISTRO/setup.bash && ./build/rclcpp_async/test/benchmark/bench_sleep"
./run.sh bash -c "source /opt/ros/\$ROS_DISTRO/setup.bash && ./build/rclcpp_async/test/benchmark/bench_pubsub"
./run.sh bash -c "source /opt/ros/\$ROS_DISTRO/setup.bash && ./build/rclcpp_async/test/benchmark/bench_service"
./run.sh bash -c "source /opt/ros/\$ROS_DISTRO/setup.bash && ./build/rclcpp_async/test/benchmark/bench_concurrent"
```

## Results

### Environment

| Item | Value |
|------|-------|
| CPU | Intel Core i9-14900K (24 cores) |
| RAM | 64 GB |
| OS | Ubuntu 24.04.1 LTS |
| Kernel | 6.14.0-37-generic |
| Compiler | GCC 13.3.0 |
| ROS | Jazzy |
| RMW | rmw_fastrtps_cpp (default) |
| Build | Release (`-DCMAKE_BUILD_TYPE=Release`) |

### Sleep Overhead

Overhead = measured duration - requested duration (us). N=50 per data point.

| Duration | `sleep_for` | callback | coroutine | coro - callback |
|----------|------------|----------|-----------|----------------|
| 1 ms | 51.9 | 36.6 | 41.3 | +4.7 |
| 5 ms | 57.0 | 40.7 | 41.2 | +0.4 |
| 10 ms | 55.8 | 38.2 | 39.9 | +1.7 |
| 50 ms | 76.5 | 44.8 | 41.5 | -3.4 |
| 100 ms | 69.8 | 45.4 | 39.8 | -5.6 |

- `sleep_for`: `std::this_thread::sleep_for`
- callback: `rclcpp::Node::create_wall_timer`
- coroutine: `co_await ctx.sleep()`
- Coroutine overhead is ~0-5 us vs callback

### Pub/Sub Latency

Publish-to-receive latency on a single node (us). N=100, warmup=10.

| Method | Mean | Min | Max |
|--------|------|-----|-----|
| callback | 3.5 | 3.1 | 5.8 |
| coroutine | 3.8 | 3.6 | 6.3 |

- callback: `create_subscription` with callback function
- coroutine: `co_await stream->next()` via `TopicStream`
- Coroutine overhead is ~0.3-0.6 us vs callback

### Service Call Latency

Request-to-response round-trip latency (us). N=100, warmup=10.

| Method | Mean | Min | Max |
|--------|------|-----|-----|
| callback | 9.3 | 7.9 | 14.5 |
| coroutine | 10.5 | 9.7 | 18.6 |

- callback: `async_send_request` with callback
- coroutine: `co_await ctx.send_request()`
- Coroutine overhead is ~1 us vs callback

### Concurrent Tasks

Wall-clock time (us) for N tasks sleeping simultaneously.

#### Single fire (repeats=1)

| Tasks | Duration | callback | coro/sleep |
|-------|----------|----------|------------|
| 100 | 1 ms | 1,255 | 1,216 |
| 500 | 1 ms | 4,009 | 4,089 |
| 1000 | 1 ms | 13,875 | 14,219 |
| 5000 | 1 ms | 326,418 | 311,274 |
| 100 | 10 ms | 10,301 | 10,362 |
| 500 | 10 ms | 12,144 | 11,947 |
| 1000 | 10 ms | 15,861 | 15,531 |
| 5000 | 10 ms | 309,932 | 310,664 |
| 100 | 100 ms | 100,370 | 100,293 |
| 500 | 100 ms | 102,065 | 102,574 |
| 1000 | 100 ms | 105,434 | 105,687 |
| 5000 | 100 ms | 310,042 | 315,420 |

- callback: periodic `create_wall_timer` with one-shot guard
- coro/sleep: `co_await ctx.sleep()` (creates a new timer each call)
- Virtually identical performance for single-fire tasks

#### Repeated fire (repeats=3)

| Tasks | Duration | callback | coro/sleep | coro/timer |
|-------|----------|----------|------------|------------|
| 100 | 1 ms | 3,306 | 3,478 | 3,217 |
| 500 | 1 ms | 7,318 | 17,907 | 7,064 |
| 1000 | 1 ms | 34,013 | 97,695 | 34,466 |
| 5000 | 1 ms | 548,041 | 1,760,863 | 542,178 |
| 100 | 10 ms | 109,813 | 149,637 | 89,421 |
| 500 | 10 ms | 192,154 | 360,985 | 187,553 |
| 1000 | 10 ms | 282,532 | 688,481 | 283,395 |
| 5000 | 10 ms | 1,512,884 | 4,289,087 | 1,482,912 |
| 100 | 100 ms | 471,350 | 491,167 | 476,378 |
| 500 | 100 ms | 626,474 | 911,163 | 635,733 |
| 1000 | 100 ms | 870,773 | 1,472,468 | 866,974 |
| 5000 | 100 ms | 2,911,061 | 7,072,643 | 2,901,263 |

- callback: periodic `create_wall_timer`
- coro/sleep: `co_await ctx.sleep()` — creates/destroys a timer on every call
- coro/timer: `co_await timer->next()` via `TimerStream` — reuses a persistent timer
- **coro/timer matches callback performance**. coro/sleep degrades at high task counts due to repeated timer creation/destruction overhead.
