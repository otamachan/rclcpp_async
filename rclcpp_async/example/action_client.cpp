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

#include <example_interfaces/action/fibonacci.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "rclcpp_async/rclcpp_async.hpp"

using Fibonacci = example_interfaces::action::Fibonacci;

rclcpp_async::Task<void> run(rclcpp_async::CoContext & ctx)
{
  auto client = rclcpp_action::create_client<Fibonacci>(ctx.node(), "fibonacci");

  RCLCPP_INFO(ctx.node()->get_logger(), "Waiting for action server...");
  auto wait_result = co_await ctx.wait_for_action(client, std::chrono::seconds(10));
  if (!wait_result.ok()) {
    RCLCPP_ERROR(ctx.node()->get_logger(), "Action server not available");
    co_return;
  }
  RCLCPP_INFO(ctx.node()->get_logger(), "Action server found");

  Fibonacci::Goal goal;
  goal.order = 8;

  RCLCPP_INFO(ctx.node()->get_logger(), "Sending goal: order=%d", goal.order);
  auto goal_result = co_await ctx.send_goal<Fibonacci>(client, goal);
  if (!goal_result.ok()) {
    RCLCPP_ERROR(ctx.node()->get_logger(), "Goal failed: %s", goal_result.error_msg.c_str());
    co_return;
  }

  auto stream = *goal_result.value;
  RCLCPP_INFO(ctx.node()->get_logger(), "Goal accepted, waiting for feedback...");

  while (true) {
    auto feedback = co_await stream->next();
    if (!feedback.has_value()) {
      break;
    }
    auto & seq = (*feedback)->sequence;
    RCLCPP_INFO(
      ctx.node()->get_logger(), "Feedback: sequence length=%zu, last=%d", seq.size(), seq.back());
  }

  auto result = stream->result();
  auto & seq = result.result->sequence;
  RCLCPP_INFO(
    ctx.node()->get_logger(), "Result: sequence length=%zu, last=%d", seq.size(), seq.back());
  RCLCPP_INFO(ctx.node()->get_logger(), "Done");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("action_client");
  rclcpp_async::CoContext ctx(node);

  auto task = ctx.create_task(run(ctx));

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
