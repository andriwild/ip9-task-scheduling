cmake_minimum_required(VERSION 3.8)
project(event_system_core)
set(CMAKE_CXX_STANDARD 20)

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
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(TEST_FIXTURES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")

  # EventQueue tests
  ament_add_gtest(test_event_queue test/test_event_queue.cpp)
  target_compile_definitions(test_event_queue PRIVATE
    TEST_FIXTURES_DIR="${TEST_FIXTURES_DIR}"
  )

  # ConfigLoader tests
  ament_add_gtest(test_config_loader test/test_config_loader.cpp)
  target_link_libraries(test_config_loader nlohmann_json::nlohmann_json)
  target_compile_definitions(test_config_loader PRIVATE
    TEST_FIXTURES_DIR="${TEST_FIXTURES_DIR}"
  )

  # MissionManager tests
  ament_add_gtest(test_mission_manager test/test_mission_manager.cpp)
  ament_target_dependencies(test_mission_manager rclcpp)
  target_compile_definitions(test_mission_manager PRIVATE
    TEST_FIXTURES_DIR="${TEST_FIXTURES_DIR}"
  )

  # Scheduler tests (with MockPathPlanner)
  ament_add_gtest(test_scheduler
    test/test_scheduler.cpp
    src/model/event.cpp
    src/model/robot.cpp
    src/model/robot_state.cpp
  )
  ament_target_dependencies(test_scheduler rclcpp behaviortree_cpp)
  target_compile_definitions(test_scheduler PRIVATE
    TEST_FIXTURES_DIR="${TEST_FIXTURES_DIR}"
  )

  # Event execute tests (with MockSimContext)
  ament_add_gtest(test_events
    test/test_events.cpp
    src/model/event.cpp
    src/model/robot.cpp
    src/model/robot_state.cpp
  )
  ament_target_dependencies(test_events rclcpp behaviortree_cpp)
  target_compile_definitions(test_events PRIVATE
    TEST_FIXTURES_DIR="${TEST_FIXTURES_DIR}"
  )

  # Integration tests (real BT + SimulationContext, MockPathPlanner)
  ament_add_gtest(test_integration
    test/test_integration.cpp
    src/model/event.cpp
    src/model/event_queue.cpp
    src/model/robot.cpp
    src/model/robot_state.cpp
    src/model/context.cpp
  )
  target_link_libraries(test_integration nlohmann_json::nlohmann_json)
  ament_target_dependencies(test_integration rclcpp behaviortree_cpp)
  target_compile_definitions(test_integration PRIVATE
    TEST_FIXTURES_DIR="${TEST_FIXTURES_DIR}"
  )
endif()

ament_package()
