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
#include <queue>
#include <rclcpp/detail/add_guard_condition_to_rcl_wait_set.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp_async/cancellation_token.hpp"
#include "rclcpp_async/channel.hpp"
#include "rclcpp_async/event.hpp"
#include "rclcpp_async/goal_context.hpp"
#include "rclcpp_async/goal_stream.hpp"
#include "rclcpp_async/mutex.hpp"
#include "rclcpp_async/result.hpp"
#include "rclcpp_async/send_request_awaiter.hpp"
#include "rclcpp_async/sleep_awaiter.hpp"
#include "rclcpp_async/task.hpp"
#include "rclcpp_async/timer_stream.hpp"
#include "rclcpp_async/topic_stream.hpp"
#include "rclcpp_async/wait_for_action_awaiter.hpp"
#include "rclcpp_async/wait_for_service_awaiter.hpp"

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

  void set_on_ready_callback(std::function<void(size_t, int)>) override {}

  void clear_on_ready_callback() override {}

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

class CoContext
{
  std::shared_ptr<DrainWaitable> drain_;
  rclcpp::Node::SharedPtr node_;

public:
  explicit CoContext(rclcpp::Node::SharedPtr node) : node_(std::move(node))
  {
    drain_ = std::make_shared<DrainWaitable>();
    node_->get_node_waitables_interface()->add_waitable(drain_, nullptr);
  }

  // Post a callback to be executed on the executor thread (thread-safe).
  void post(std::function<void()> fn) { drain_->post(std::move(fn)); }

  // Resume a coroutine directly. Call only from the executor thread.
  void resume(std::coroutine_handle<> h) { h.resume(); }

  rclcpp::Node::SharedPtr node() { return node_; }

  template <typename T>
  [[nodiscard]] Task<T> create_task(Task<T> task)
  {
    task.started_ = true;
    task.handle.resume();
    return task;
  }

  template <typename CallbackT>
  [[nodiscard]] auto create_task(CallbackT && callback)
  {
    return create_task(callback());
  }

  // --- Awaiter factory methods ---

  WaitForServiceAwaiter wait_for_service(
    rclcpp::ClientBase::SharedPtr client, std::chrono::nanoseconds timeout = 5s)
  {
    return WaitForServiceAwaiter{*this,   std::move(client), timeout, nullptr,
                                 nullptr, nullptr,           {},      false};
  }

  template <typename ServiceT>
  SendRequestAwaiter<ServiceT> send_request(
    typename rclcpp::Client<ServiceT>::SharedPtr client,
    typename ServiceT::Request::SharedPtr request)
  {
    return SendRequestAwaiter<ServiceT>{*this, std::move(client), std::move(request), nullptr, {},
                                        false};
  }

  SleepAwaiter sleep(std::chrono::nanoseconds duration)
  {
    return SleepAwaiter{*this, duration, nullptr, nullptr, {}, false};
  }

  template <typename ActionT>
  WaitForActionAwaiter<ActionT> wait_for_action(
    std::shared_ptr<rclcpp_action::Client<ActionT>> client, std::chrono::nanoseconds timeout = 5s)
  {
    return WaitForActionAwaiter<ActionT>{*this,   std::move(client), timeout, nullptr,
                                         nullptr, nullptr,           {},      false};
  }

  template <typename MsgT>
  std::shared_ptr<TopicStream<MsgT>> subscribe(const std::string & topic, const rclcpp::QoS & qos)
  {
    auto stream = std::make_shared<TopicStream<MsgT>>(*this);
    stream->sub_ = node_->template create_subscription<MsgT>(
      topic, qos, [s = stream](typename MsgT::SharedPtr msg) {
        if (s->closed_) {
          return;
        }
        s->queue_.push(std::move(msg));
        if (s->waiter_) {
          auto w = s->waiter_;
          s->waiter_ = nullptr;
          s->ctx_.resume(w);
        }
      });
    return stream;
  }

  template <typename ActionT>
  SendGoalAwaiter<ActionT> send_goal(
    typename rclcpp_action::Client<ActionT>::SharedPtr client, typename ActionT::Goal goal)
  {
    return SendGoalAwaiter<ActionT>{*this, std::move(client), std::move(goal), nullptr, nullptr, {},
                                    false};
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
    return node_->template create_service<ServiceT>(name, std::move(handler));
  }

  std::shared_ptr<TimerStream> create_timer(std::chrono::nanoseconds period)
  {
    auto stream = std::make_shared<TimerStream>(*this);
    stream->timer_ = node_->create_wall_timer(period, [s = stream, this]() {
      if (s->waiter_) {
        auto w = s->waiter_;
        s->waiter_ = nullptr;
        resume(w);
      } else {
        s->pending_++;
      }
    });
    return stream;
  }

  template <typename ActionT, typename CallbackT, typename GoalCbT, typename CancelCbT>
  typename rclcpp_action::Server<ActionT>::SharedPtr create_action_server(
    const std::string & name, CallbackT && callback, GoalCbT && goal_callback,
    CancelCbT && cancel_callback)
  {
    using GoalHandleT = rclcpp_action::ServerGoalHandle<ActionT>;

    return rclcpp_action::create_server<ActionT>(
      node_, name, std::forward<GoalCbT>(goal_callback), std::forward<CancelCbT>(cancel_callback),
      [cb = std::forward<CallbackT>(callback)](
        const std::shared_ptr<GoalHandleT> goal_handle) mutable {
        [](CallbackT & cb, std::shared_ptr<GoalHandleT> gh) -> SpawnedTask {
          auto aborted = std::make_shared<bool>(false);
          GoalContext<ActionT> goal(gh, aborted);
          auto result_value = co_await cb(std::move(goal));
          auto result = std::make_shared<typename ActionT::Result>(std::move(result_value));
          if (gh->is_canceling()) {
            gh->canceled(result);
          } else if (*aborted) {
            gh->abort(result);
          } else {
            gh->succeed(result);
          }
        }(cb, goal_handle);
      });
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

template <typename DonePred, typename CancelAction>
inline void register_cancel(
  CancellationToken * token, CoContext & ctx, std::coroutine_handle<> h, DonePred is_done,
  CancelAction action)
{
  if (!token) {
    return;
  }
  token->on_cancel([&ctx, h, is_done, action]() {
    ctx.post([&ctx, h, is_done, action]() {
      if (is_done()) {
        return;
      }
      action();
      ctx.resume(h);
    });
  });
}

// ============================================================
// Awaiter await_suspend implementations (need full CoContext)
// ============================================================

inline void WaitForServiceAwaiter::await_suspend(std::coroutine_handle<> h)
{
  auto finish = [this, h](Result<void> r) {
    if (done) {
      return;
    }
    done = true;
    poll_timer->cancel();
    deadline_timer->cancel();
    result = std::move(r);
    ctx.resume(h);
  };

  poll_timer = ctx.node()->create_wall_timer(100ms, [this, finish]() {
    if (client->service_is_ready()) {
      finish(Result<void>::Ok());
    }
  });

  deadline_timer =
    ctx.node()->create_wall_timer(timeout, [finish]() { finish(Result<void>::Timeout()); });

  register_cancel(
    token, ctx, h, [this]() { return done; },
    [this]() {
      done = true;
      poll_timer->cancel();
      deadline_timer->cancel();
      result = Result<void>::Cancelled();
    });
}

template <typename ServiceT>
void SendRequestAwaiter<ServiceT>::await_suspend(std::coroutine_handle<> h)
{
  client->async_send_request(
    request, [this, h](typename rclcpp::Client<ServiceT>::SharedFuture future) {
      if (done) {
        return;
      }
      done = true;
      result = Result<Response>::Ok(future.get());
      ctx.resume(h);
    });

  register_cancel(
    token, ctx, h, [this]() { return done; },
    [this]() {
      done = true;
      result = Result<Response>::Cancelled();
    });
}

inline void SleepAwaiter::await_suspend(std::coroutine_handle<> h)
{
  auto finish = [this, h](Result<void> r) {
    if (done) {
      return;
    }
    done = true;
    timer->cancel();
    result = std::move(r);
    ctx.resume(h);
  };

  timer = ctx.node()->create_wall_timer(duration, [finish]() { finish(Result<void>::Ok()); });

  register_cancel(
    token, ctx, h, [this]() { return done; },
    [this]() {
      done = true;
      timer->cancel();
      result = Result<void>::Cancelled();
    });
}

template <typename ActionT>
void WaitForActionAwaiter<ActionT>::await_suspend(std::coroutine_handle<> h)
{
  auto finish = [this, h](Result<void> r) {
    if (done) {
      return;
    }
    done = true;
    poll_timer->cancel();
    deadline_timer->cancel();
    result = std::move(r);
    ctx.resume(h);
  };

  poll_timer = ctx.node()->create_wall_timer(100ms, [this, finish]() {
    if (client->action_server_is_ready()) {
      finish(Result<void>::Ok());
    }
  });

  deadline_timer =
    ctx.node()->create_wall_timer(timeout, [finish]() { finish(Result<void>::Timeout()); });

  if (token) {
    token->on_cancel(
      [this, finish]() { ctx.post([finish]() { finish(Result<void>::Cancelled()); }); });
  }
}

template <typename MsgT>
void TopicStream<MsgT>::NextAwaiter::await_suspend(std::coroutine_handle<> h)
{
  stream.waiter_ = h;
  register_cancel(
    token, stream.ctx_, h, [this, h]() { return stream.waiter_ != h; },
    [this]() {
      stream.waiter_ = nullptr;
      cancelled = true;
    });
}

template <typename MsgT>
void TopicStream<MsgT>::close()
{
  closed_ = true;
  if (waiter_) {
    auto w = waiter_;
    waiter_ = nullptr;
    ctx_.resume(w);
  }
}

template <typename ActionT>
void GoalStream<ActionT>::resume_waiter(std::coroutine_handle<> h)
{
  ctx_.resume(h);
}

template <typename ActionT>
void GoalStream<ActionT>::NextAwaiter::await_suspend(std::coroutine_handle<> h)
{
  stream.waiter_ = h;
  register_cancel(
    token, stream.ctx_, h, [this, h]() { return stream.waiter_ != h; },
    [this]() {
      stream.waiter_ = nullptr;
      cancelled = true;
    });
}

template <typename ActionT>
void GoalStream<ActionT>::CancelAwaiter::await_suspend(std::coroutine_handle<> h)
{
  stream.client_->async_cancel_goal(
    stream.goal_handle_, [this, h](typename CancelResponse::SharedPtr resp) {
      response = std::move(resp);
      stream.ctx_.resume(h);
    });
}

template <typename ActionT>
void SendGoalAwaiter<ActionT>::await_suspend(std::coroutine_handle<> h)
{
  stream = std::make_shared<GoalStream<ActionT>>(ctx);

  typename rclcpp_action::Client<ActionT>::SendGoalOptions options;

  options.feedback_callback = [this](auto, const auto & feedback) {
    GoalEvent<ActionT> event;
    event.type = GoalEvent<ActionT>::Type::kFeedback;
    event.feedback = feedback;
    stream->push(std::move(event));
  };

  options.result_callback = [this](const auto & wrapped_result) {
    GoalEvent<ActionT> event;
    event.type = GoalEvent<ActionT>::Type::kComplete;
    event.result = wrapped_result;
    stream->push(std::move(event));
  };

  options.goal_response_callback = [this, h](const auto & goal_handle) {
    if (done) {
      return;
    }
    done = true;
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
    ctx.post([&ctx = ctx, h]() { ctx.resume(h); });
  };

  client->async_send_goal(goal, options);

  register_cancel(
    token, ctx, h, [this]() { return done; },
    [this]() {
      done = true;
      result = Result<std::shared_ptr<GoalStream<ActionT>>>::Cancelled();
    });
}

// ============================================================
// Event / Mutex implementations (need full CoContext)
// ============================================================

inline void Event::WaitAwaiter::await_suspend(std::coroutine_handle<> h)
{
  active = std::make_shared<bool>(true);
  event.waiters_.push({h, active});
  auto a = active;
  register_cancel(token, event.ctx_, h, [a]() { return !*a; }, [a]() { *a = false; });
}

inline void Event::set()
{
  set_ = true;
  while (!waiters_.empty()) {
    auto w = waiters_.front();
    waiters_.pop();
    if (*w.active) {
      *w.active = false;
      ctx_.resume(w.handle);
    }
  }
}

inline void Mutex::LockAwaiter::await_suspend(std::coroutine_handle<> h)
{
  active = std::make_shared<bool>(true);
  mutex.waiters_.push({h, active});
  auto a = active;
  register_cancel(token, mutex.ctx_, h, [a]() { return !*a; }, [a]() { *a = false; });
}

inline void Mutex::unlock()
{
  while (!waiters_.empty()) {
    auto w = waiters_.front();
    waiters_.pop();
    if (*w.active) {
      *w.active = false;
      ctx_.resume(w.handle);
      return;
    }
  }
  locked_ = false;
}

// ============================================================
// TimerStream await_suspend (need full CoContext)
// ============================================================

inline void TimerStream::NextAwaiter::await_suspend(std::coroutine_handle<> h)
{
  stream.waiter_ = h;
  register_cancel(
    token, stream.ctx_, h, [this, h]() { return stream.waiter_ != h; },
    [this]() {
      stream.waiter_ = nullptr;
      cancelled = true;
    });
}

// ============================================================
// Channel implementations (need full CoContext)
// ============================================================

template <typename T>
bool Channel<T>::NextAwaiter::await_suspend(std::coroutine_handle<> h)
{
  {
    std::lock_guard lock(ch.mutex_);
    if (!ch.queue_.empty() || ch.closed_) {
      return false;  // don't suspend, data already available
    }
    ch.waiter_ = h;
  }  // release ch.mutex_ before on_cancel to avoid deadlock
  if (token) {
    token->on_cancel([this, h]() {
      std::coroutine_handle<> w;
      {
        std::lock_guard lock(ch.mutex_);
        if (ch.waiter_ != h) {
          return;
        }
        w = ch.waiter_;
        ch.waiter_ = nullptr;
      }
      if (w) {
        cancelled = true;
        ch.ctx_.post([w]() { w.resume(); });
      }
    });
  }
  return true;
}

template <typename T>
void Channel<T>::send(T value)
{
  std::coroutine_handle<> w;
  {
    std::lock_guard lock(mutex_);
    if (closed_) {
      return;
    }
    queue_.push(std::move(value));
    w = waiter_;
    waiter_ = nullptr;
  }
  if (w) {
    ctx_.post([w]() { w.resume(); });
  }
}

template <typename T>
void Channel<T>::close()
{
  std::coroutine_handle<> w;
  {
    std::lock_guard lock(mutex_);
    closed_ = true;
    w = waiter_;
    waiter_ = nullptr;
  }
  if (w) {
    ctx_.post([w]() { w.resume(); });
  }
}

}  // namespace rclcpp_async
