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

// Nested service call demo: demonstrates that coroutine-based async I/O
// avoids deadlock even with chained calls on a single-threaded executor.
//
// Call flow:
//   client task → fibonacci action server
//     → co_await send_request("validate", ...)
//       → validate service server
//         → co_await send_request("slow_square", ...)
//           → slow_square service server (co_await sleep 1s)

#include <chrono>
#include <example_interfaces/action/fibonacci.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>

#include "rclcpp_async/rclcpp_async.hpp"

using namespace std::chrono_literals;  // NOLINT(build/namespaces)
using SetBool = std_srvs::srv::SetBool;
using Fibonacci = example_interfaces::action::Fibonacci;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("nested_demo");
  rclcpp_async::CoContext ctx(node);

  // --- Service: slow_square (leaf service) ---
  // Simulates a slow computation: sleeps 1s then responds.
  auto slow_square_srv = ctx.create_service<SetBool>(
    "slow_square",
    [&ctx](SetBool::Request::SharedPtr req)
      -> rclcpp_async::Task<SetBool::Response> {
      auto logger = ctx.node()->get_logger();
      RCLCPP_INFO(logger, "[slow_square] received (data=%s)",
        req->data ? "true" : "false");
      co_await ctx.sleep(1s);
      SetBool::Response resp;
      resp.success = req->data;
      resp.message = "slow_square done";
      RCLCPP_INFO(logger, "[slow_square] responding");
      co_return resp;
    });
  RCLCPP_INFO(node->get_logger(), "Service 'slow_square' ready");

  // --- Service: validate (middle service) ---
  // Calls slow_square internally, then responds.
  auto slow_square_client =
    node->create_client<SetBool>("slow_square");
  auto validate_srv = ctx.create_service<SetBool>(
    "validate",
    [&ctx, slow_square_client](SetBool::Request::SharedPtr req)
      -> rclcpp_async::Task<SetBool::Response> {
      auto logger = ctx.node()->get_logger();
      RCLCPP_INFO(logger, "[validate] received (data=%s)",
        req->data ? "true" : "false");

      // Nested call to slow_square
      auto inner_req = std::make_shared<SetBool::Request>();
      inner_req->data = req->data;
      RCLCPP_INFO(logger, "[validate] calling slow_square...");
      auto result = co_await ctx.send_request<SetBool>(
        slow_square_client, inner_req);

      SetBool::Response resp;
      if (result.ok()) {
        auto inner_resp = result.value.value();
        resp.success = inner_resp->success;
        resp.message =
          "validate ok (inner: " + inner_resp->message + ")";
        RCLCPP_INFO(logger, "[validate] slow_square returned: %s",
          inner_resp->message.c_str());
      } else {
        resp.success = false;
        resp.message = "validate failed: slow_square error";
        RCLCPP_ERROR(logger, "[validate] slow_square call failed");
      }
      co_return resp;
    });
  RCLCPP_INFO(node->get_logger(), "Service 'validate' ready");

  // --- Action: fibonacci ---
  // Calls validate service before computing fibonacci sequence.
  auto validate_client =
    node->create_client<SetBool>("validate");
  auto action_server = ctx.create_action_server<Fibonacci>(
    "fibonacci",
    [&ctx, validate_client](
      rclcpp_async::GoalContext<Fibonacci> goal)
      -> rclcpp_async::Task<Fibonacci::Result> {
      auto logger = ctx.node()->get_logger();
      RCLCPP_INFO(logger, "[fibonacci] received goal: order=%d",
        goal.goal().order);

      // Nested call to validate (which itself calls slow_square)
      auto req = std::make_shared<SetBool::Request>();
      req->data = true;
      RCLCPP_INFO(logger, "[fibonacci] calling validate...");
      auto val_result = co_await ctx.send_request<SetBool>(
        validate_client, req);
      if (val_result.ok()) {
        auto resp = val_result.value.value();
        RCLCPP_INFO(logger, "[fibonacci] validate returned: %s",
          resp->message.c_str());
      } else {
        RCLCPP_ERROR(logger, "[fibonacci] validate call failed");
      }

      // Compute fibonacci with feedback
      Fibonacci::Feedback feedback;
      feedback.sequence = {0, 1};

      for (int i = 2; i < goal.goal().order; i++) {
        if (goal.is_canceling()) {
          RCLCPP_INFO(logger, "[fibonacci] canceled at step %d", i);
          break;
        }
        auto n = feedback.sequence.size();
        feedback.sequence.push_back(
          feedback.sequence[n - 1] + feedback.sequence[n - 2]);
        goal.publish_feedback(feedback);
        RCLCPP_INFO(logger, "[fibonacci] feedback: seq[%d] = %d",
          i, feedback.sequence.back());
        co_await ctx.sleep(200ms);
      }

      Fibonacci::Result result;
      result.sequence = feedback.sequence;
      RCLCPP_INFO(logger, "[fibonacci] done, length=%zu",
        result.sequence.size());
      co_return result;
    });
  RCLCPP_INFO(node->get_logger(), "Action 'fibonacci' ready");

  // --- Client task: send fibonacci goal ---
  auto client_task =
    ctx.create_task([&ctx]() -> rclcpp_async::Task<void> {
      auto logger = ctx.node()->get_logger();
      auto action_client =
        rclcpp_action::create_client<Fibonacci>(
          ctx.node(), "fibonacci");

      RCLCPP_INFO(logger, "[client] waiting for action server...");
      auto wait_result =
        co_await ctx.wait_for_action(action_client, 10s);
      if (!wait_result.ok()) {
        RCLCPP_ERROR(logger, "[client] server not available");
        co_return;
      }

      Fibonacci::Goal goal;
      goal.order = 8;
      RCLCPP_INFO(logger, "[client] sending goal: order=%d",
        goal.order);
      auto goal_result =
        co_await ctx.send_goal<Fibonacci>(action_client, goal);
      if (!goal_result.ok()) {
        RCLCPP_ERROR(logger, "[client] goal failed: %s",
          goal_result.error_msg.c_str());
        co_return;
      }

      auto stream = *goal_result.value;
      RCLCPP_INFO(logger, "[client] goal accepted");

      while (true) {
        auto feedback = co_await stream->next();
        if (!feedback.has_value()) {
          break;
        }
        auto & seq = (*feedback)->sequence;
        RCLCPP_INFO(logger, "[client] feedback: length=%zu, last=%d",
          seq.size(), seq.back());
      }

      auto result = stream->result();
      auto & seq = result.result->sequence;
      RCLCPP_INFO(logger, "[client] result: length=%zu, last=%d",
        seq.size(), seq.back());
      RCLCPP_INFO(logger, "[client] demo completed successfully!");
      RCLCPP_INFO(logger,
        "[client] no deadlock on single-threaded executor");

      rclcpp::shutdown();
    });

  rclcpp::spin(node);
  return 0;
}
