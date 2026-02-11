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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "rclcpp_async/rclcpp_async.hpp"

using SetBool = std_srvs::srv::SetBool;

rclcpp_async::Task<void> run(rclcpp_async::CoContext & ctx)
{
  auto client = ctx.node()->create_client<SetBool>("set_bool");

  RCLCPP_INFO(ctx.node()->get_logger(), "Waiting for service...");
  auto wait_result = co_await ctx.wait_for_service(client, std::chrono::seconds(10));
  if (!wait_result.ok()) {
    RCLCPP_ERROR(ctx.node()->get_logger(), "Service not available");
    co_return;
  }
  RCLCPP_INFO(ctx.node()->get_logger(), "Service found");

  bool value = true;
  for (int i = 0; i < 5; i++) {
    auto req = std::make_shared<SetBool::Request>();
    req->data = value;

    RCLCPP_INFO(ctx.node()->get_logger(), "Sending request: %s", value ? "true" : "false");
    auto resp = co_await ctx.send_request<SetBool>(client, req);
    RCLCPP_INFO(
      ctx.node()->get_logger(), "Response: success=%s, message='%s'",
      resp->success ? "true" : "false", resp->message.c_str());

    value = !value;
    co_await ctx.sleep(std::chrono::seconds(1));
  }

  RCLCPP_INFO(ctx.node()->get_logger(), "Done");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("service_client");
  rclcpp_async::CoContext ctx(node);

  auto task = ctx.create_task(run(ctx));

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
