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

// Regression test for the TfBuffer internal TF listener node inheriting a
// global `__node` remap.
//
// A component container is launched with e.g.
//     --ros-args -r __node:=pfr_nav_container
// which is a *global* node-name remap. Before the fix, every TfBuffer created
// its internal listener with default NodeOptions (use_global_arguments == true)
// and a fixed name "_tf_listener", so the global remap renamed each one to the
// container name. With one such node per TfBuffer-using component, this produced
// duplicate node names and "Publisher already registered for node name" rosout
// warnings.
//
// The fix gives the internal node use_global_arguments(false) (so it ignores the
// CLI `__node` remap) and a name derived from the host node (so concurrent
// listeners do not collide). This test reproduces the global remap and asserts
// both properties via the node graph.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rclcpp_async/rclcpp_async.hpp"

using namespace rclcpp_async;          // NOLINT(build/namespaces)
using namespace std::chrono_literals;  // NOLINT(build/namespaces)

TEST(TfBufferNodeName, IgnoresGlobalNodeRemapAndIsUnique)
{
  // Simulate a container launched with a global `__node` remap.
  const char * argv[] = {
    "test_tf_buffer_node_name", "--ros-args", "-r", "__node:=globally_remapped"};
  const int argc = static_cast<int>(sizeof(argv) / sizeof(argv[0]));
  rclcpp::init(argc, argv);

  // Host node with a known name, itself shielded from the global remap so its
  // name is deterministic for the assertions below.
  auto host = std::make_shared<rclcpp::Node>(
    "host_under_test", rclcpp::NodeOptions().use_global_arguments(false));
  CoContext ctx(*host);
  TfBuffer tf_buffer(ctx);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(host);

  // Let the graph discover the internal TF listener node.
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  bool found_unique = false;
  bool found_remapped = false;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(20ms);

    const std::vector<std::string> names = host->get_node_names();
    found_unique =
      std::find(names.begin(), names.end(), "/host_under_test_tf_listener") != names.end();
    // The internal node must NOT have adopted the global __node remap.
    found_remapped =
      std::find(names.begin(), names.end(), "/globally_remapped") != names.end();
    if (found_unique) {
      break;
    }
  }

  EXPECT_TRUE(found_unique)
    << "internal TF listener node should be uniquely named after the host node";
  EXPECT_FALSE(found_remapped)
    << "internal TF listener node must ignore the global __node remap";

  rclcpp::shutdown();
}
