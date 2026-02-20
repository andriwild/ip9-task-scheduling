cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 20)
project(event_system_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(backward_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/SetSystemState.srv"
    "srv/SetSystemConfig.srv"
    "msg/SystemConfig.msg"
    "msg/TimelineEvent.msg"
    "msg/TimelineStateChange.msg"
    "msg/TimelineReset.msg"
    "msg/TimelineMeeting.msg"
    "msg/MetricsReport.msg"
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
