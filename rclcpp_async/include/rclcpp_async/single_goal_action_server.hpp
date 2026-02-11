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

#include <functional>
#include <memory>
#include <stop_token>
#include <string>
#include <utility>

#include "rclcpp_async/co_context.hpp"

namespace rclcpp_async
{

/// Create an action server that executes only one goal at a time.
/// When a new goal arrives while another is executing, the previous goal is
/// preempted (cancelled via CancelledException) and the new goal starts
/// only after the previous goal has fully completed.
template <typename ActionT, typename CallbackT>
auto create_single_goal_action_server(
  CoContext & ctx, const std::string & name, CallbackT && callback) ->
  typename rclcpp_action::Server<ActionT>::SharedPtr
{
  using Result = typename ActionT::Result;
  auto current_stop = std::make_shared<std::stop_source>();
  auto gate = std::make_shared<std::shared_ptr<Event>>();

  return ctx.create_action_server<ActionT>(
    name,
    [cb = std::forward<CallbackT>(callback), current_stop, gate,
     &ctx](GoalContext<ActionT> goal) mutable -> Task<Result> {
      // Capture preempt token BEFORE gate wait so rapid A→B→C
      // correctly detects that B was preempted while queued.
      auto preempt_token = current_stop->get_token();

      // Wait for the previous goal to finish (cancel + abort).
      auto prev = *gate;
      auto done = std::make_shared<Event>(ctx);
      *gate = done;
      if (prev) {
        co_await prev->wait();
      }

      // If preempted while waiting on the gate, abort immediately.
      if (preempt_token.stop_requested()) {
        done->set();
        throw CancelledException{};
      }

      auto inner = cb(std::move(goal));
      StopCb preempt_cb(preempt_token, [&inner]() { inner.cancel(); });
      try {
        auto result = co_await std::move(inner);
        done->set();
        co_return result;
      } catch (...) {
        done->set();
        throw;
      }
    },
    [current_stop](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal>) {
      current_stop->request_stop();
      *current_stop = std::stop_source{};
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [current_stop](std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>) {
      current_stop->request_stop();
      return rclcpp_action::CancelResponse::ACCEPT;
    });
}

}  // namespace rclcpp_async
