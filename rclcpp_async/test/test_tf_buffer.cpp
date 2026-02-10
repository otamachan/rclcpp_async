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

#include <gtest/gtest.h>

#ifdef TF2_ROS_VERSION_GTE_0_45
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#else
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#endif

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "rclcpp_async/rclcpp_async.hpp"

using namespace rclcpp_async;          // NOLINT(build/namespaces)
using namespace std::chrono_literals;  // NOLINT(build/namespaces)

static tf2_ros::StaticTransformBroadcaster make_static_broadcaster(
  const rclcpp::Node::SharedPtr & node)
{
#ifdef TF2_ROS_VERSION_GTE_0_45
  return tf2_ros::StaticTransformBroadcaster(
    tf2_ros::StaticTransformBroadcaster::RequiredInterfaces(
      node->get_node_parameters_interface(), node->get_node_topics_interface()));
#else
  return tf2_ros::StaticTransformBroadcaster(node);
#endif
}

static geometry_msgs::msg::TransformStamped make_transform(
  const std::string & parent, const std::string & child,
  const rclcpp::Time & stamp = rclcpp::Time(0))
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;
  tf.transform.rotation.w = 1.0;
  tf.transform.translation.x = 1.0;
  return tf;
}

class TfBufferTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_tf_node");
    ctx_ = std::make_unique<CoContext>(node_);
    executor_.add_node(node_);
    tf_buffer_ = std::make_unique<TfBuffer>(*ctx_);
  }

  void TearDown() override
  {
    tf_buffer_.reset();
    ctx_.reset();
    node_.reset();
  }

  void spin_until_done(Task<void> & task, std::chrono::seconds timeout = 5s)
  {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!task.handle.done() && std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(1ms);
    }
  }

  void wait_for_tf_listener(std::chrono::milliseconds duration = 500ms)
  {
    auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(10ms);
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<CoContext> ctx_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::unique_ptr<TfBuffer> tf_buffer_;
};

TEST_F(TfBufferTest, ImmediateLookupStaticTf)
{
  // Publish a static transform
  auto broadcaster = make_static_broadcaster(node_);
  broadcaster.sendTransform(make_transform("map", "odom"));

  // Wait for the TF listener thread to receive it
  wait_for_tf_listener();

  geometry_msgs::msg::TransformStamped result;
  bool done = false;

  auto coro = [&]() -> Task<void> {
    // Should resolve immediately in await_ready() since TF is already available
    result = co_await tf_buffer_->lookup_transform("map", "odom", rclcpp::Time(0));
    done = true;
  };

  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  ASSERT_TRUE(done);
  EXPECT_EQ(result.header.frame_id, "map");
  EXPECT_EQ(result.child_frame_id, "odom");
  EXPECT_DOUBLE_EQ(result.transform.translation.x, 1.0);
}

TEST_F(TfBufferTest, SyncLookupReturnsOptional)
{
  // Before any transform is published, lookup returns nullopt
  auto empty = tf_buffer_->lookup_transform("map", "odom");
  EXPECT_FALSE(empty.has_value());

  // Publish a static transform
  auto broadcaster = make_static_broadcaster(node_);
  broadcaster.sendTransform(make_transform("map", "odom"));

  wait_for_tf_listener();

  // Now lookup returns a value
  auto result = tf_buffer_->lookup_transform("map", "odom");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->header.frame_id, "map");
  EXPECT_EQ(result->child_frame_id, "odom");
  EXPECT_DOUBLE_EQ(result->transform.translation.x, 1.0);
}

TEST_F(TfBufferTest, AsyncLookupWaitsForTf)
{
  geometry_msgs::msg::TransformStamped result;
  bool done = false;

  auto coro = [&]() -> Task<void> {
    // Lookup will suspend because TF is not yet available
    result = co_await tf_buffer_->lookup_transform("world", "sensor", rclcpp::Time(0));
    done = true;
  };

  auto task = ctx_->create_task(coro());

  // Spin a bit without publishing — should not complete
  for (int i = 0; i < 50; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_FALSE(done);

  // Now publish the transform
  auto broadcaster = make_static_broadcaster(node_);
  broadcaster.sendTransform(make_transform("world", "sensor"));

  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  ASSERT_TRUE(done);
  EXPECT_EQ(result.header.frame_id, "world");
  EXPECT_EQ(result.child_frame_id, "sensor");
}

TEST_F(TfBufferTest, TimeoutWhenNoTf)
{
  bool timed_out = false;

  auto coro = [&]() -> Task<void> {
    auto res =
      co_await ctx_->wait_for(tf_buffer_->lookup_transform("a", "b", rclcpp::Time(0)), 500ms);
    timed_out = res.timeout();
  };

  auto task = ctx_->create_task(coro());
  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  EXPECT_TRUE(timed_out);
}

TEST_F(TfBufferTest, CancellationRemovesPendingRequest)
{
  bool was_cancelled = false;

  auto coro = [&]() -> Task<void> {
    try {
      co_await tf_buffer_->lookup_transform("x", "y", rclcpp::Time(0));
    } catch (const CancelledException &) {
      was_cancelled = true;
    }
  };

  auto task = coro();
  auto running = ctx_->create_task(std::move(task));

  // Let it suspend on the lookup
  for (int i = 0; i < 30; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  running.cancel();
  spin_until_done(running);

  ASSERT_TRUE(running.handle.done());
  EXPECT_TRUE(was_cancelled);
}

TEST_F(TfBufferTest, MultipleConcurrentLookups)
{
  geometry_msgs::msg::TransformStamped r1, r2;
  bool done = false;

  auto coro = [&]() -> Task<void> {
    auto [t1, t2] = co_await when_all(
      tf_buffer_->lookup_transform("map", "odom", rclcpp::Time(0)),
      tf_buffer_->lookup_transform("odom", "base", rclcpp::Time(0)));
    r1 = t1;
    r2 = t2;
    done = true;
  };

  auto task = ctx_->create_task(coro());

  // Spin a bit, then publish both transforms
  for (int i = 0; i < 30; i++) {
    executor_.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  auto broadcaster = make_static_broadcaster(node_);
  broadcaster.sendTransform(make_transform("map", "odom"));
  broadcaster.sendTransform(make_transform("odom", "base"));

  spin_until_done(task);

  ASSERT_TRUE(task.handle.done());
  ASSERT_TRUE(done);
  EXPECT_EQ(r1.child_frame_id, "odom");
  EXPECT_EQ(r2.child_frame_id, "base");
}
