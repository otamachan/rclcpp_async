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

#ifdef TF2_ROS_VERSION_GTE_0_45
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/buffer_interface.hpp>
#else
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#endif

#include <coroutine>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <stop_token>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/qos.hpp>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp_async/cancelled_exception.hpp"
#include "rclcpp_async/co_context.hpp"

namespace rclcpp_async
{

class TfBuffer
{
public:
  struct PendingRequest
  {
    std::string target_frame;
    std::string source_frame;
    tf2::TimePoint time;
    std::coroutine_handle<> handle;
    std::shared_ptr<bool> active;
    geometry_msgs::msg::TransformStamped result;
  };

  explicit TfBuffer(CoContext & ctx)
  : ctx_(ctx),
    tf_node_(std::make_shared<rclcpp::Node>(
      "_tf_listener", ctx.node().get_namespace(),
      rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(
        false))),
    buffer_(std::make_shared<tf2_ros::Buffer>(ctx.node().get_clock()))
  {
    auto cb = [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
      on_tf_message(std::move(msg), false);
    };
    auto static_cb = [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
      on_tf_message(std::move(msg), true);
    };
    sub_tf_ = tf_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", tf2_ros::DynamicListenerQoS(), std::move(cb));
    sub_tf_static_ = tf_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", tf2_ros::StaticListenerQoS(), std::move(static_cb));

    tf_executor_.add_node(tf_node_);
    tf_thread_ = std::thread([this]() { tf_executor_.spin(); });
  }

  ~TfBuffer()
  {
    tf_executor_.cancel();
    if (tf_thread_.joinable()) {
      tf_thread_.join();
    }
  }

  TfBuffer(const TfBuffer &) = delete;
  TfBuffer & operator=(const TfBuffer &) = delete;
  TfBuffer(TfBuffer &&) = delete;
  TfBuffer & operator=(TfBuffer &&) = delete;

  std::optional<geometry_msgs::msg::TransformStamped> lookup_transform(
    const std::string & target_frame, const std::string & source_frame) const
  {
    if (!buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero)) {
      return std::nullopt;
    }
    return buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  }

  struct LookupTransformAwaiter
  {
    TfBuffer & tf;
    std::string target_frame;
    std::string source_frame;
    tf2::TimePoint time;
    std::stop_token token;
    std::unique_ptr<StopCb> cancel_cb_;
    bool cancelled = false;
    std::shared_ptr<PendingRequest> request;

    void set_token(std::stop_token t) { token = std::move(t); }

    bool await_ready()
    {
      if (token.stop_requested()) {
        cancelled = true;
        return true;
      }
      if (tf.buffer_->canTransform(target_frame, source_frame, time)) {
        request = std::make_shared<PendingRequest>();
        request->result = tf.buffer_->lookupTransform(target_frame, source_frame, time);
        return true;
      }
      return false;
    }

    void await_suspend(std::coroutine_handle<> h)
    {
      request = std::make_shared<PendingRequest>();
      request->target_frame = target_frame;
      request->source_frame = source_frame;
      request->time = time;
      request->handle = h;
      request->active = std::make_shared<bool>(true);
      tf.add_pending(request);

      register_cancel(
        cancel_cb_, token, tf.ctx_, h, [r = request]() { return !*r->active; },
        [this, &tf_buf = tf, r = request]() {
          tf_buf.remove_pending(r);
          *r->active = false;
          cancelled = true;
        });
    }

    geometry_msgs::msg::TransformStamped await_resume()
    {
      cancel_cb_.reset();
      if (cancelled) {
        throw CancelledException{};
      }
      return request->result;
    }
  };

  LookupTransformAwaiter lookup_transform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time)
  {
    return LookupTransformAwaiter{*this, target_frame, source_frame, tf2_ros::fromRclcpp(time),
                                  {},    {},           false,        nullptr};
  }

  void add_pending(std::shared_ptr<PendingRequest> req)
  {
    std::lock_guard lock(mutex_);
    pending_.push_back(std::move(req));
  }

  void remove_pending(const std::shared_ptr<PendingRequest> & req)
  {
    std::lock_guard lock(mutex_);
    std::erase(pending_, req);
  }

private:
  void on_tf_message(tf2_msgs::msg::TFMessage::ConstSharedPtr msg, bool is_static)
  {
    for (const auto & transform : msg->transforms) {
      try {
        buffer_->setTransform(transform, "default_authority", is_static);
      } catch (const tf2::TransformException &) {
        // Ignore malformed transforms
      }
    }
    check_pending();
  }

  void check_pending()
  {
    std::lock_guard lock(mutex_);
    std::erase_if(pending_, [this](const std::shared_ptr<PendingRequest> & req) {
      if (!*req->active) {
        return true;
      }
      if (buffer_->canTransform(req->target_frame, req->source_frame, req->time)) {
        try {
          req->result = buffer_->lookupTransform(req->target_frame, req->source_frame, req->time);
        } catch (const tf2::TransformException &) {
          return false;
        }
        *req->active = false;
        auto h = req->handle;
        ctx_.post([h]() { h.resume(); });
        return true;
      }
      return false;
    });
  }

  CoContext & ctx_;

  rclcpp::Node::SharedPtr tf_node_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;

  rclcpp::executors::SingleThreadedExecutor tf_executor_;
  std::thread tf_thread_;

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_static_;

  std::mutex mutex_;
  std::vector<std::shared_ptr<PendingRequest>> pending_;
};

}  // namespace rclcpp_async
