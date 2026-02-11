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

/// \brief Component node example: pub, sub, service server, and service client
///        all as coroutine tasks. Launch two instances with remapping to
///        demonstrate cross-node communication on a shared executor.

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>

#include "rclcpp_async/rclcpp_async.hpp"

using namespace std::chrono_literals;  // NOLINT(build/namespaces)
using SetBool = std_srvs::srv::SetBool;

namespace rclcpp_async_example
{

class ComponentDemo : public rclcpp::Node
{
public:
  explicit ComponentDemo(const rclcpp::NodeOptions & options)
  : Node("component_demo", options), ctx_(*this)
  {
    service_ = ctx_.create_service<SetBool>("~/serve", [this](auto req) { return serve(req); });
    pub_task_ = ctx_.create_task(publish_loop());
    sub_task_ = ctx_.create_task(subscribe_loop());
    client_task_ = ctx_.create_task(client_loop());
    RCLCPP_INFO(get_logger(), "Started");
  }

private:
  // Service handler
  rclcpp_async::Task<SetBool::Response> serve(SetBool::Request::SharedPtr req)
  {
    RCLCPP_INFO(get_logger(), "[server] got %s", req->data ? "true" : "false");
    co_await ctx_.sleep(100ms);
    SetBool::Response resp;
    resp.success = req->data;
    resp.message = std::string(get_name()) + ": " + (req->data ? "ack" : "nak");
    RCLCPP_INFO(get_logger(), "[server] reply: %s", resp.message.c_str());
    co_return resp;
  }

  // Publisher: ~/pub
  rclcpp_async::Task<void> publish_loop()
  {
    auto pub = create_publisher<std_msgs::msg::String>("~/pub", 10);
    auto timer = ctx_.create_timer(1s);
    int count = 0;

    while (true) {
      co_await timer->next();
      std_msgs::msg::String msg;
      msg.data = std::string(get_name()) + " #" + std::to_string(count++);
      RCLCPP_INFO(get_logger(), "[pub] %s", msg.data.c_str());
      pub->publish(msg);
    }
  }

  // Subscriber: ~/sub
  rclcpp_async::Task<void> subscribe_loop()
  {
    auto stream = ctx_.subscribe<std_msgs::msg::String>("~/sub", 10);

    while (true) {
      auto msg = co_await stream->next();
      if (!msg.has_value()) {
        break;
      }
      RCLCPP_INFO(get_logger(), "[sub] %s", (*msg)->data.c_str());
    }
  }

  // Service client: ~/call
  rclcpp_async::Task<void> client_loop()
  {
    auto client = create_client<SetBool>("~/call");

    auto wait_result = co_await ctx_.wait_for_service(client, 10s);
    if (!wait_result.ok()) {
      RCLCPP_ERROR(get_logger(), "[client] service not available");
      co_return;
    }

    bool value = true;
    for (int i = 0; i < 5; i++) {
      co_await ctx_.sleep(2s);

      auto req = std::make_shared<SetBool::Request>();
      req->data = value;
      RCLCPP_INFO(get_logger(), "[client] send %s", value ? "true" : "false");

      auto resp = co_await ctx_.send_request<SetBool>(client, req);
      RCLCPP_INFO(get_logger(), "[client] got: %s", resp->message.c_str());

      value = !value;
    }
    RCLCPP_INFO(get_logger(), "[client] done");
  }

  rclcpp_async::CoContext ctx_;
  rclcpp::Service<SetBool>::SharedPtr service_;
  rclcpp_async::Task<void> pub_task_;
  rclcpp_async::Task<void> sub_task_;
  rclcpp_async::Task<void> client_task_;
};

}  // namespace rclcpp_async_example

RCLCPP_COMPONENTS_REGISTER_NODE(rclcpp_async_example::ComponentDemo)
