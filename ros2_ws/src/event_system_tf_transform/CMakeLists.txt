cmake_minimum_required(VERSION 3.8)
project(event_system_tf_transform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gz-math8 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_executable(transform_node src/transform_node.cpp)

ament_target_dependencies(transform_node
  rclcpp
  geometry_msgs
  tf2_msgs
  tf2_ros
)

target_link_libraries(transform_node
  gz-math8::gz-math8
  gz-transport13::gz-transport13
)

install(TARGETS transform_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
