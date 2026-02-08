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

#include <chrono>
#include <example_interfaces/action/fibonacci.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>

#include "rclcpp_async/rclcpp_async.hpp"

using namespace std::chrono_literals;  // NOLINT(build/namespaces)
using rclcpp_async::CoContext;         // NOLINT(build/namespaces)
using rclcpp_async::GoalContext;       // NOLINT(build/namespaces)
using rclcpp_async::Task;              // NOLINT(build/namespaces)
using SetBool = std_srvs::srv::SetBool;
using Fibonacci = example_interfaces::action::Fibonacci;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("demo_server");
  CoContext ctx(node);

  // Service: set_bool (coroutine)
  auto service = ctx.create_service<SetBool>(
    "set_bool", [&node](SetBool::Request::SharedPtr req) -> Task<SetBool::Response> {
      SetBool::Response resp;
      resp.success = req->data;
      resp.message = req->data ? "enabled" : "disabled";
      RCLCPP_INFO(
        node->get_logger(), "Request: %s -> %s", req->data ? "true" : "false",
        resp.message.c_str());
      co_return resp;
    });
  RCLCPP_INFO(node->get_logger(), "Service 'set_bool' ready");

  // Action: fibonacci (coroutine)
  auto action_server = ctx.create_action_server<Fibonacci>(
    "fibonacci", [&ctx, &node](GoalContext<Fibonacci> goal) -> Task<Fibonacci::Result> {
      RCLCPP_INFO(node->get_logger(), "Received goal: order=%d", goal.goal().order);

      Fibonacci::Feedback feedback;
      feedback.sequence = {0, 1};

      for (int i = 2; i < goal.goal().order; i++) {
        if (goal.is_canceling()) {
          RCLCPP_INFO(node->get_logger(), "Goal canceled at step %d", i);
          break;
        }
        auto n = feedback.sequence.size();
        feedback.sequence.push_back(feedback.sequence[n - 1] + feedback.sequence[n - 2]);
        goal.publish_feedback(feedback);
        RCLCPP_INFO(node->get_logger(), "Feedback: sequence[%d] = %d", i, feedback.sequence.back());
        co_await ctx.sleep(500ms);
      }

      Fibonacci::Result result;
      result.sequence = feedback.sequence;
      co_return result;
    });
  RCLCPP_INFO(node->get_logger(), "Action 'fibonacci' ready");

  // Publisher: topic_a (1s interval, timer stream)
  auto task_a = ctx.create_task([&ctx]() -> Task<void> {
    auto n = ctx.node();
    auto pub = n->create_publisher<std_msgs::msg::String>("topic_a", 10);
    auto timer = ctx.create_timer(1s);
    int count = 0;
    while (true) {
      co_await timer->next();
      auto msg = std_msgs::msg::String();
      msg.data = "Hello from A: " + std::to_string(count++);
      RCLCPP_INFO(n->get_logger(), "Publishing to topic_a: '%s'", msg.data.c_str());
      pub->publish(msg);
    }
  });

  // Publisher: topic_b (700ms interval, timer stream)
  auto task_b = ctx.create_task([&ctx]() -> Task<void> {
    auto n = ctx.node();
    auto pub = n->create_publisher<std_msgs::msg::String>("topic_b", 10);
    auto timer = ctx.create_timer(700ms);
    int count = 0;
    while (true) {
      co_await timer->next();
      auto msg = std_msgs::msg::String();
      msg.data = "Hello from B: " + std::to_string(count++);
      RCLCPP_INFO(n->get_logger(), "Publishing to topic_b: '%s'", msg.data.c_str());
      pub->publish(msg);
    }
  });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
