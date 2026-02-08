cmake_minimum_required(VERSION 3.14)
project(rclcpp_async)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fcoroutines)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)

if(CMAKE_COMPILER_IS_GNUCXX)
  target_compile_options(${PROJECT_NAME} INTERFACE -fcoroutines)
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify ament_cmake_cppcheck)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(example_interfaces REQUIRED)

  # Examples
  add_executable(demo_server example/demo_server.cpp)
  target_link_libraries(demo_server ${PROJECT_NAME})
  ament_target_dependencies(demo_server std_msgs std_srvs example_interfaces)

  add_executable(subscriber example/subscriber.cpp)
  target_link_libraries(subscriber ${PROJECT_NAME})
  ament_target_dependencies(subscriber std_msgs)

  add_executable(service_client example/service_client.cpp)
  target_link_libraries(service_client ${PROJECT_NAME})
  ament_target_dependencies(service_client std_srvs)

  add_executable(action_client example/action_client.cpp)
  target_link_libraries(action_client ${PROJECT_NAME})
  ament_target_dependencies(action_client example_interfaces)

  install(TARGETS demo_server subscriber service_client action_client
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )

  # Level 1: Pure unit tests (no ROS2 runtime)
  ament_add_gtest(test_result test/test_result.cpp)
  target_link_libraries(test_result ${PROJECT_NAME})

  ament_add_gtest(test_cancellation_token test/test_cancellation_token.cpp)
  target_link_libraries(test_cancellation_token ${PROJECT_NAME})

  ament_add_gtest(test_task test/test_task.cpp)
  target_link_libraries(test_task ${PROJECT_NAME})

  # Level 2: ROS2 integration tests (isolated domain ID for parallel safety)
  ament_add_ros_isolated_gtest(test_sleep test/test_sleep.cpp)
  target_link_libraries(test_sleep ${PROJECT_NAME})

  ament_add_ros_isolated_gtest(test_wait_for_service test/test_wait_for_service.cpp)
  target_link_libraries(test_wait_for_service ${PROJECT_NAME})
  ament_target_dependencies(test_wait_for_service std_srvs)

  ament_add_ros_isolated_gtest(test_wait_for_action test/test_wait_for_action.cpp)
  target_link_libraries(test_wait_for_action ${PROJECT_NAME})
  ament_target_dependencies(test_wait_for_action example_interfaces)

  ament_add_ros_isolated_gtest(test_send_request test/test_send_request.cpp)
  target_link_libraries(test_send_request ${PROJECT_NAME})
  ament_target_dependencies(test_send_request std_srvs)

  ament_add_ros_isolated_gtest(test_topic_stream test/test_topic_stream.cpp)
  target_link_libraries(test_topic_stream ${PROJECT_NAME})
  ament_target_dependencies(test_topic_stream std_msgs)

  ament_add_ros_isolated_gtest(test_goal_stream test/test_goal_stream.cpp)
  target_link_libraries(test_goal_stream ${PROJECT_NAME})
  ament_target_dependencies(test_goal_stream example_interfaces)

  ament_add_ros_isolated_gtest(test_create_service test/test_create_service.cpp)
  target_link_libraries(test_create_service ${PROJECT_NAME})
  ament_target_dependencies(test_create_service std_srvs)

  ament_add_ros_isolated_gtest(test_event test/test_event.cpp)
  target_link_libraries(test_event ${PROJECT_NAME})

  ament_add_ros_isolated_gtest(test_mutex test/test_mutex.cpp)
  target_link_libraries(test_mutex ${PROJECT_NAME})

  ament_add_ros_isolated_gtest(test_timer_stream test/test_timer_stream.cpp)
  target_link_libraries(test_timer_stream ${PROJECT_NAME})

  ament_add_ros_isolated_gtest(test_create_action_server test/test_create_action_server.cpp)
  target_link_libraries(test_create_action_server ${PROJECT_NAME})
  ament_target_dependencies(test_create_action_server example_interfaces)
endif()

ament_package()
