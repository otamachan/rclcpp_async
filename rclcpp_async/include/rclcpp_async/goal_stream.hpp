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

#include <coroutine>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <stop_token>
#include <utility>

#include "rclcpp_async/cancelled_exception.hpp"
#include "rclcpp_async/executor.hpp"
#include "rclcpp_async/result.hpp"

namespace rclcpp_async
{

class CoContext;

template <typename ActionT>
struct GoalEvent
{
  using Feedback = std::shared_ptr<const typename ActionT::Feedback>;
  using WrappedResult = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;

  enum class Type
  {
    kFeedback,
    kComplete
  };
  Type type{};
  Feedback feedback;
  WrappedResult result{};
};

template <typename ActionT>
class GoalStream
{
  using Event = GoalEvent<ActionT>;
  using WrappedResult = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;

  using GoalHandlePtr = typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr;
  using ActionClientPtr = typename rclcpp_action::Client<ActionT>::SharedPtr;

  CoContext & ctx_;
  std::queue<Event> queue_;
  std::coroutine_handle<> waiter_;
  size_t max_depth_;
  bool completed_ = false;
  bool auto_cancel_on_stop_ = true;
  WrappedResult final_result_;
  GoalHandlePtr goal_handle_;
  ActionClientPtr client_;

  template <typename A>
  friend struct SendGoalAwaiter;

  void push(Event event)
  {
    bool is_complete = (event.type == Event::Type::kComplete);
    if (is_complete) {
      final_result_ = std::move(event.result);
      completed_ = true;
    }
    queue_.push(std::move(event));
    while (queue_.size() > max_depth_ && queue_.front().type != Event::Type::kComplete) {
      queue_.pop();
    }
    if (waiter_) {
      auto h = waiter_;
      waiter_ = nullptr;
      resume_waiter(h);
    }
  }

  void resume_waiter(std::coroutine_handle<> h);

public:
  explicit GoalStream(CoContext & ctx, size_t max_depth = kDefaultStreamDepth)
  : ctx_(ctx), max_depth_(max_depth)
  {
  }

  struct NextAwaiter
  {
    using Feedback = std::shared_ptr<const typename ActionT::Feedback>;

    GoalStream & stream;
    std::stop_token token;
    std::shared_ptr<StopCb> cancel_cb_;
    bool cancelled = false;

    void set_token(std::stop_token t) { token = std::move(t); }

    bool await_ready()
    {
      if (token.stop_requested()) {
        cancelled = true;
        return true;
      }
      return !stream.queue_.empty();
    }

    void await_suspend(std::coroutine_handle<> h);

    std::optional<Feedback> await_resume()
    {
      cancel_cb_.reset();
      if (cancelled) {
        throw CancelledException{};
      }
      auto & event = stream.queue_.front();
      std::optional<Feedback> ret;
      if (event.type == Event::Type::kFeedback) {
        ret = std::move(event.feedback);
      }
      stream.queue_.pop();
      return ret;
    }
  };

  NextAwaiter next() { return NextAwaiter{*this, {}, {}, false}; }

  using CancelResponse = typename ActionT::Impl::CancelGoalService::Response;

  struct CancelAwaiter
  {
    GoalStream & stream;
    typename CancelResponse::SharedPtr response;

    bool await_ready() { return false; }

    void await_suspend(std::coroutine_handle<> h);

    typename CancelResponse::SharedPtr await_resume() { return std::move(response); }
  };

  CancelAwaiter cancel_goal() { return CancelAwaiter{*this, nullptr}; }

  void set_auto_cancel_on_stop(bool v) { auto_cancel_on_stop_ = v; }

  WrappedResult result() const { return final_result_; }
};

template <typename ActionT>
struct SendGoalAwaiter
{
  // Mutable state is held via shared_ptr so that callbacks registered on
  // the rclcpp_action::Client (notably goal_response_callback) can safely
  // observe `done` even if the awaiter (and its coroutine frame) has been
  // destroyed after the awaiting coroutine was cancelled before the goal
  // response arrived.
  struct State
  {
    bool done = false;
    bool cancelled = false;
    std::shared_ptr<GoalStream<ActionT>> stream;
    Result<std::shared_ptr<GoalStream<ActionT>>> result;
  };

  CoContext & ctx;
  typename rclcpp_action::Client<ActionT>::SharedPtr client;
  typename ActionT::Goal goal;
  size_t max_depth;
  std::stop_token token;
  std::shared_ptr<StopCb> cancel_cb_;
  std::shared_ptr<State> state_;

  void set_token(std::stop_token t) { token = std::move(t); }

  bool await_ready()
  {
    if (token.stop_requested()) {
      state_->cancelled = true;
      return true;
    }
    return false;
  }

  void await_suspend(std::coroutine_handle<> h);

  Result<std::shared_ptr<GoalStream<ActionT>>> await_resume()
  {
    cancel_cb_.reset();
    if (state_->cancelled) {
      throw CancelledException{};
    }
    return std::move(state_->result);
  }
};

}  // namespace rclcpp_async
