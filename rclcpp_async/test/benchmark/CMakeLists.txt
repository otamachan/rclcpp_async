add_executable(bench_sleep bench_sleep.cpp)
target_link_libraries(bench_sleep ${PROJECT_NAME})

add_executable(bench_pubsub bench_pubsub.cpp)
target_link_libraries(bench_pubsub ${PROJECT_NAME} ${std_msgs_TARGETS})

add_executable(bench_service bench_service.cpp)
target_link_libraries(bench_service ${PROJECT_NAME} ${std_srvs_TARGETS})

add_executable(bench_concurrent bench_concurrent.cpp)
target_link_libraries(bench_concurrent ${PROJECT_NAME})
