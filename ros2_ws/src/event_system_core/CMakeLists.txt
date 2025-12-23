cmake_minimum_required(VERSION 3.8)
project(event_system_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_AUTOMOC ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(gz-math8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(event_system_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets Charts Sql REQUIRED)

file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "src/*.h" "src/*.hpp")

add_executable(event_system_core ${SOURCES} ${HEADERS})

target_include_directories(event_system_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(event_system_core
  Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Charts Qt5::Sql
  nlohmann_json::nlohmann_json
  gz-math8::gz-math8
  gz-transport13::gz-transport13
  atomic
)

ament_target_dependencies(event_system_core
  rclcpp
  tf2_ros
  geometry_msgs
  std_msgs
  behaviortree_cpp
  visualization_msgs
  event_system_msgs
  nav2_msgs
)

install(TARGETS event_system_core
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
