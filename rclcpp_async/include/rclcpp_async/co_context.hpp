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

#pragma once

#include <rclcpp/version.h>

#include <chrono>
#include <coroutine>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <rclcpp/detail/add_guard_condition_to_rcl_wait_set.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "rclcpp_async/cancelled_exception.hpp"
#include "rclcpp_async/channel.hpp"
#include "rclcpp_async/event.hpp"
#include "rclcpp_async/executor.hpp"
#include "rclcpp_async/goal_context.hpp"
#include "rclcpp_async/goal_stream.hpp"
#include "rclcpp_async/mutex.hpp"
#include "rclcpp_async/result.hpp"
#include "rclcpp_async/send_request_awaiter.hpp"
#include "rclcpp_async/sleep_awaiter.hpp"
#include "rclcpp_async/task.hpp"
#include "rclcpp_async/timer_stream.hpp"
#include "rclcpp_async/topic_stream.hpp"
#include "rclcpp_async/when_any.hpp"

namespace rclcpp_async
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

class DrainWaitable : public rclcpp::Waitable
{
  rclcpp::GuardCondition gc_;
  std::mutex mutex_;
  std::queue<std::function<void()>> queue_;
  bool ready_ = false;

public:
  void post(std::function<void()> fn)
  {
    {
      std::lock_guard lock(mutex_);
      queue_.push(std::move(fn));
    }
    gc_.trigger();
  }

  size_t get_number_of_ready_guard_conditions() override { return 1; }

  void add_to_wait_set(rcl_wait_set_t & wait_set) override
  {
    rclcpp::detail::add_guard_condition_to_rcl_wait_set(wait_set, gc_);
  }

  bool is_ready(const rcl_wait_set_t &) override
  {
    std::lock_guard lock(mutex_);
    ready_ = !queue_.empty();
    return ready_;
  }

  std::shared_ptr<void> take_data() override { return nullptr; }

  std::shared_ptr<void> take_data_by_entity_id(size_t) override { return nullptr; }

  void set_on_ready_callback(std::function<void(size_t, int)> callback) override
  {
    // Events-based executors only learn this waitable is ready via this
    // callback, not by polling the wait set. Forward guard condition triggers.
    // GuardCondition replays triggers raced ahead of registration, so no loss.
    gc_.set_on_trigger_callback([callback](size_t count) { callback(count, 0); });
  }

  void clear_on_ready_callback() override { gc_.set_on_trigger_callback(nullptr); }

#if RCLCPP_VERSION_GTE(30, 0, 0)
  std::vector<std::shared_ptr<rclcpp::TimerBase>> get_timers() const override { return {}; }
#endif

  void execute(const std::shared_ptr<void> &) override
  {
    std::function<void()> fn;
    while (true) {
      {
        std::lock_guard lock(mutex_);
        if (queue_.empty()) {
          return;
        }
        fn = std::move(queue_.front());
        queue_.pop();
      }
      fn();
    }
  }
};

class CoContext : public Executor
{
  std::shared_ptr<DrainWaitable> drain_;
  rclcpp::Node & node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::SubscriptionOptions subscription_options() const
  {
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    return options;
  }

public:
  // Everything the context creates (its drain waitable, timers, subscriptions,
  // services and action servers) joins callback_group, so coroutines resume
  // in that group. nullptr keeps the node's default group. The group must be
  // MutuallyExclusive: the waiters the context keeps are not thread-safe.
  //
  // Clients passed to send_request / send_goal resume in their own group, and
  // Event::set, Mutex::unlock, TopicStream::close and create_task resume on
  // the calling thread, so call or create those from the same group to keep
  // a coroutine serialized with the rest of it.
  explicit CoContext(rclcpp::Node & node, rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
  : node_(node), callback_group_(std::move(callback_group))
  {
    if (
      callback_group_ && callback_group_->type() != rclcpp::CallbackGroupType::MutuallyExclusive) {
      throw std::invalid_argument("CoContext needs a MutuallyExclusive callback group");
    }
    drain_ = std::make_shared<DrainWaitable>();
    node_.get_node_waitables_interface()->add_waitable(drain_, callback_group_);
  }

  CoContext(const CoContext &) = delete;
  CoContext & operator=(const CoContext &) = delete;

  // Post a callback to be executed on the executor thread (thread-safe).
  void post(std::function<void()> fn) override { drain_->post(std::move(fn)); }

  // Resume a coroutine directly. Call only from the executor thread.
  void resume(std::coroutine_handle<> h) override { h.resume(); }

  rclcpp::Node & node() { return node_; }

  rclcpp::CallbackGroup::SharedPtr callback_group() const { return callback_group_; }

  template <typename T>
  [[nodiscard]] Task<T> create_task(Task<T> task)
  {
    task.started_ = true;
    task.handle.resume();
    return task;
  }

  // The callable is moved into the task's frame: a lambda's captures must
  // outlive the caller's full-expression, as the task resumes after it.
  template <typename CallbackT>
  [[nodiscard]] auto create_task(CallbackT && callback)
  {
    return create_task([](std::decay_t<CallbackT> cb) -> decltype(cb()) {
      co_return co_await cb();
    }(std::forward<CallbackT>(callback)));
  }

  // --- Awaiter factory methods ---

  Task<Result<void>> wait_for_service(
    rclcpp::ClientBase::SharedPtr client, std::chrono::nanoseconds timeout = 5s)
  {
    co_return co_await poll_until(
      [client]() { return client->service_is_ready(); }, 100ms, timeout);
  }

  template <typename ServiceT>
  SendRequestAwaiter<ServiceT> send_request(
    typename rclcpp::Client<ServiceT>::SharedPtr client,
    typename ServiceT::Request::SharedPtr request)
  {
    return SendRequestAwaiter<ServiceT>{
      *this, std::move(client), std::move(request), {}, {}, false, nullptr, nullptr};
  }

  SleepAwaiter sleep(std::chrono::nanoseconds duration)
  {
    return SleepAwaiter{*this, duration, nullptr, {}, {}, false, nullptr};
  }

  template <typename ActionT>
  Task<Result<void>> wait_for_action(
    std::shared_ptr<rclcpp_action::Client<ActionT>> client, std::chrono::nanoseconds timeout = 5s)
  {
    co_return co_await poll_until(
      [client]() { return client->action_server_is_ready(); }, 100ms, timeout);
  }

  template <typename MsgT>
  std::shared_ptr<TopicStream<MsgT>> subscribe(const std::string & topic, const rclcpp::QoS & qos)
  {
    auto stream = std::make_shared<TopicStream<MsgT>>(*this, qos.depth());
    // The subscription is owned by the stream, so the callback holds it weakly:
    // dropping the stream releases the subscription with it.
    stream->sub_ = node_.template create_subscription<MsgT>(
      topic, qos,
      [weak = std::weak_ptr(stream)](std::shared_ptr<const MsgT> msg) {
        auto s = weak.lock();
        if (!s || s->closed_) {
          return;
        }
        s->queue_.push(std::move(msg));
        while (s->queue_.size() > s->max_depth_) {
          s->queue_.pop();
        }
        resume_now(s->ctx_, std::exchange(s->waiter_, {}));
      },
      subscription_options());
    return stream;
  }

  template <typename ActionT>
  SendGoalAwaiter<ActionT> send_goal(
    typename rclcpp_action::Client<ActionT>::SharedPtr client, typename ActionT::Goal goal,
    size_t max_depth = kDefaultStreamDepth)
  {
    return SendGoalAwaiter<ActionT>{
      *this, std::move(client), std::move(goal), max_depth, {}, {}, false, nullptr, {}, nullptr};
  }

  template <typename ServiceT, typename CallbackT>
  typename rclcpp::Service<ServiceT>::SharedPtr create_service(
    const std::string & name, CallbackT && callback)
  {
    std::function<void(
      std::shared_ptr<rclcpp::Service<ServiceT>>, std::shared_ptr<rmw_request_id_t>,
      typename ServiceT::Request::SharedPtr)>
      handler = [cb = std::forward<CallbackT>(callback)](
                  std::shared_ptr<rclcpp::Service<ServiceT>> service_handle,
                  std::shared_ptr<rmw_request_id_t> request_id,
                  typename ServiceT::Request::SharedPtr request) mutable {
        [](
          CallbackT & cb, std::shared_ptr<rclcpp::Service<ServiceT>> srv,
          std::shared_ptr<rmw_request_id_t> req_id,
          typename ServiceT::Request::SharedPtr req) -> SpawnedTask {
          auto response = co_await cb(std::move(req));
          srv->send_response(*req_id, response);
        }(cb, std::move(service_handle), std::move(request_id), std::move(request));
      };
    return node_.template create_service<ServiceT>(
      name, std::move(handler), rclcpp::ServicesQoS(), callback_group_);
  }

  std::shared_ptr<TimerStream> create_timer(std::chrono::nanoseconds period)
  {
    auto stream = std::make_shared<TimerStream>(*this);
    // As with subscribe(): dropping the stream stops and releases the timer.
    stream->timer_ = node_.create_wall_timer(
      period,
      [weak = std::weak_ptr(stream), this]() {
        auto s = weak.lock();
        if (!s || s->closed_) {
          return;
        }
        if (!resume_now(*this, std::exchange(s->waiter_, {}))) {
          s->pending_++;
        }
      },
      callback_group_);
    return stream;
  }

  template <typename Pred>
  Task<Result<void>> poll_until(
    Pred pred, std::chrono::nanoseconds interval, std::chrono::nanoseconds timeout)
  {
    if (pred()) co_return Result<void>::Ok();
    auto timer = create_timer(interval);
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      co_await timer->next();
      if (pred()) co_return Result<void>::Ok();
    }
    co_return Result<void>::Timeout();
  }

  template <typename T>
  Task<Result<T>> wait_for(Task<T> task, std::chrono::nanoseconds timeout)
  {
    auto timeout_task = [this, timeout]() -> Task<void> { co_await sleep(timeout); };
    auto result = co_await when_any(std::move(task), timeout_task());
    if (result.index() == 0) {
      if constexpr (std::is_void_v<T>) {
        co_return Result<void>::Ok();
      } else {
        co_return Result<T>::Ok(std::get<0>(result));
      }
    } else {
      co_return Result<T>::Timeout();
    }
  }

  template <typename Awaitable>
  auto wait_for(Awaitable awaitable, std::chrono::nanoseconds timeout)
  {
    return wait_for(as_task(std::move(awaitable)), timeout);
  }

  template <typename ActionT, typename CallbackT, typename GoalCbT, typename CancelCbT>
  typename rclcpp_action::Server<ActionT>::SharedPtr create_action_server(
    const std::string & name, CallbackT && callback, GoalCbT && goal_callback,
    CancelCbT && cancel_callback)
  {
    using GoalHandleT = rclcpp_action::ServerGoalHandle<ActionT>;

    return rclcpp_action::create_server<ActionT>(
      node_.get_node_base_interface(), node_.get_node_clock_interface(),
      node_.get_node_logging_interface(), node_.get_node_waitables_interface(), name,
      std::forward<GoalCbT>(goal_callback), std::forward<CancelCbT>(cancel_callback),
      [cb = std::forward<CallbackT>(callback)](
        const std::shared_ptr<GoalHandleT> goal_handle) mutable {
        [](CallbackT & cb, std::shared_ptr<GoalHandleT> gh) -> SpawnedTask {
          auto aborted = std::make_shared<bool>(false);
          GoalContext<ActionT> goal(gh, aborted);
          try {
            auto result_value = co_await cb(std::move(goal));
            auto result = std::make_shared<typename ActionT::Result>(std::move(result_value));
            if (gh->is_canceling()) {
              gh->canceled(result);
            } else if (*aborted) {
              gh->abort(result);
            } else {
              gh->succeed(result);
            }
          } catch (const CancelledException &) {
            if (gh->is_canceling()) {
              gh->canceled(std::make_shared<typename ActionT::Result>());
            } else if (gh->is_active()) {
              gh->abort(std::make_shared<typename ActionT::Result>());
            }
          }
        }(cb, goal_handle);
      },
      rcl_action_server_get_default_options(), callback_group_);
  }

  template <typename ActionT, typename CallbackT>
  typename rclcpp_action::Server<ActionT>::SharedPtr create_action_server(
    const std::string & name, CallbackT && callback)
  {
    using GoalHandleT = rclcpp_action::ServerGoalHandle<ActionT>;

    return create_action_server<ActionT>(
      name, std::forward<CallbackT>(callback),
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandleT>) { return rclcpp_action::CancelResponse::ACCEPT; });
  }
};

template <typename ServiceT>
void SendRequestAwaiter<ServiceT>::await_suspend(std::coroutine_handle<> h)
{
  // The callback may outlive the awaiter, so it uses `this` only while
  // waiting_ is alive.
  waiting_ = std::make_shared<Waiting>(h);
  client->async_send_request(
    request,
    [this, w = std::weak_ptr(waiting_)](typename rclcpp::Client<ServiceT>::SharedFuture future) {
      if (auto s = w.lock()) {
        response = future.get();
        resume_now(ctx, w);
      }
    });

  register_cancel(cancel_cb_, token, ctx, waiting_, [this]() { cancelled = true; });
}

inline void SleepAwaiter::await_suspend(std::coroutine_handle<> h)
{
  waiting_ = std::make_shared<Waiting>(h);
  timer = ctx.node().create_wall_timer(
    duration,
    [this]() {
      timer->cancel();
      resume_now(ctx, waiting_);
    },
    ctx.callback_group());

  // Cancelled on the spot, so a tick after the stop request cannot win.
  cancel_cb_ = std::make_shared<StopCb>(token, [this]() {
    post_resume(ctx, waiting_, [this]() {
      timer->cancel();
      cancelled = true;
    });
  });
}

template <typename MsgT>
void TopicStream<MsgT>::NextAwaiter::await_suspend(std::coroutine_handle<> h)
{
  waiting_ = std::make_shared<Waiting>(h);
  stream.waiter_ = waiting_;
  register_cancel(cancel_cb_, token, stream.ctx_, waiting_, [this]() {
    stream.waiter_.reset();
    cancelled = true;
  });
}

template <typename MsgT>
void TopicStream<MsgT>::close()
{
  closed_ = true;
  resume_now(ctx_, std::exchange(waiter_, {}));
}

template <typename ActionT>
void GoalStream<ActionT>::resume_waiter(const std::weak_ptr<Waiting> & w)
{
  resume_now(ctx_, w);
}

template <typename ActionT>
void GoalStream<ActionT>::NextAwaiter::await_suspend(std::coroutine_handle<> h)
{
  waiting_ = std::make_shared<Waiting>(h);
  stream.waiter_ = waiting_;
  register_cancel(cancel_cb_, token, stream.ctx_, waiting_, [this]() {
    stream.waiter_.reset();
    cancelled = true;
    stream.cancel_goal_on_stop();
  });
}

template <typename ActionT>
void GoalStream<ActionT>::CancelAwaiter::await_suspend(std::coroutine_handle<> h)
{
  waiting_ = std::make_shared<Waiting>(h);
  // `this` is alive exactly while waiting_ is.
  stream.client_->async_cancel_goal(
    stream.goal_handle_,
    [this, w = std::weak_ptr(waiting_)](typename CancelResponse::SharedPtr resp) {
      if (auto s = w.lock()) {
        response = std::move(resp);
        resume_now(stream.ctx_, w);
      }
    });
}

template <typename ActionT>
void SendGoalAwaiter<ActionT>::await_suspend(std::coroutine_handle<> h)
{
  stream = std::make_shared<GoalStream<ActionT>>(ctx, max_depth);

  typename rclcpp_action::Client<ActionT>::SendGoalOptions options;

  options.feedback_callback = [s = stream](auto, const auto & feedback) {
    GoalEvent<ActionT> event;
    event.type = GoalEvent<ActionT>::Type::kFeedback;
    event.feedback = feedback;
    s->push(std::move(event));
  };

  options.result_callback = [s = stream](const auto & wrapped_result) {
    GoalEvent<ActionT> event;
    event.type = GoalEvent<ActionT>::Type::kComplete;
    event.result = wrapped_result;
    s->push(std::move(event));
  };

  // As in send_request, the callback uses `this` only while waiting_ is alive.
  waiting_ = std::make_shared<Waiting>(h);
  options.goal_response_callback = [this, w = std::weak_ptr(waiting_)](const auto & goal_handle) {
    auto s = w.lock();
    if (!s) {
      return;
    }
    if (!goal_handle) {
      result = Result<std::shared_ptr<GoalStream<ActionT>>>::Error("goal rejected");
    } else {
      stream->goal_handle_ = goal_handle;
      stream->client_ = client;
      result = Result<std::shared_ptr<GoalStream<ActionT>>>::Ok(stream);
    }
    // Use post() instead of resume() to defer coroutine resumption.
    // rclcpp_action holds goal_requests_mutex during this callback
    // (Jazzy bug: https://github.com/ros2/rclcpp/issues/2796).
    // Resuming synchronously here would deadlock if the coroutine
    // immediately calls async_send_goal again.
    post_resume(ctx, w);
  };

  client->async_send_goal(goal, options);

  register_cancel(cancel_cb_, token, ctx, waiting_, [this]() { cancelled = true; });
}

inline void TimerStream::NextAwaiter::await_suspend(std::coroutine_handle<> h)
{
  waiting_ = std::make_shared<Waiting>(h);
  stream.waiter_ = waiting_;
  // Stopped on the spot, as Sleep is.
  cancel_cb_ = std::make_shared<StopCb>(token, [this]() {
    post_resume(stream.ctx_, waiting_, [this]() {
      stream.cancel();
      cancelled = true;
    });
  });
}

}  // namespace rclcpp_async
