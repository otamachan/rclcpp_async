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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "rclcpp_async/rclcpp_async.hpp"

rclcpp_async::Task<void> listen(rclcpp_async::CoContext & ctx, std::string topic)
{
  auto stream = ctx.subscribe<std_msgs::msg::String>(topic, 10);

  while (auto msg = co_await stream->next()) {
    RCLCPP_INFO(ctx.node()->get_logger(), "[%s] %s", topic.c_str(), (*msg)->data.c_str());
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("async_subscriber");
  rclcpp_async::CoContext ctx(node);

  // 2つのトピックを並列で購読
  auto task_a = ctx.create_task(listen(ctx, "topic_a"));
  auto task_b = ctx.create_task(listen(ctx, "topic_b"));

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
