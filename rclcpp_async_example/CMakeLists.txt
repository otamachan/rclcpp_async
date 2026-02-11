cmake_minimum_required(VERSION 3.14)
project(rclcpp_async_example)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=deprecated-declarations -fcoroutines)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_async REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)

add_executable(demo_server src/demo_server.cpp)
target_link_libraries(demo_server rclcpp_async::rclcpp_async
  ${std_msgs_TARGETS} ${std_srvs_TARGETS} ${example_interfaces_TARGETS})

add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber rclcpp_async::rclcpp_async ${std_msgs_TARGETS})

add_executable(service_client src/service_client.cpp)
target_link_libraries(service_client rclcpp_async::rclcpp_async ${std_srvs_TARGETS})

add_executable(action_client src/action_client.cpp)
target_link_libraries(action_client rclcpp_async::rclcpp_async ${example_interfaces_TARGETS})

add_executable(nested_demo src/nested_demo.cpp)
target_link_libraries(nested_demo rclcpp_async::rclcpp_async
  ${std_srvs_TARGETS} ${example_interfaces_TARGETS})

add_library(component_demo SHARED src/component_demo.cpp)
target_link_libraries(component_demo rclcpp_async::rclcpp_async
  ${std_msgs_TARGETS} ${std_srvs_TARGETS})
ament_target_dependencies(component_demo rclcpp_components)
rclcpp_components_register_node(component_demo
  PLUGIN "rclcpp_async_example::ComponentDemo"
  EXECUTABLE component_demo_node)

install(TARGETS demo_server subscriber service_client action_client nested_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS component_demo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify ament_cmake_cppcheck)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
