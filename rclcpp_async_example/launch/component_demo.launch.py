# Copyright 2025 Tamaki Nishino
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Two ComponentDemo instances (alice & bob) in a single container.
#
# Remapping wires them together:
#   alice ~/pub  -> topic "alice_to_bob"  <- bob ~/sub
#   bob   ~/pub  -> topic "bob_to_alice"  <- alice ~/sub
#   alice ~/call -> bob   ~/serve
#   bob   ~/call -> alice ~/serve

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='component_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='rclcpp_async_example',
                    plugin='rclcpp_async_example::ComponentDemo',
                    name='alice',
                    remappings=[
                        ('~/pub', 'alice_to_bob'),
                        ('~/sub', 'bob_to_alice'),
                        ('~/call', '/bob/serve'),
                    ],
                ),
                ComposableNode(
                    package='rclcpp_async_example',
                    plugin='rclcpp_async_example::ComponentDemo',
                    name='bob',
                    remappings=[
                        ('~/pub', 'bob_to_alice'),
                        ('~/sub', 'alice_to_bob'),
                        ('~/call', '/alice/serve'),
                    ],
                ),
            ],
            output='screen',
        ),
    ])
