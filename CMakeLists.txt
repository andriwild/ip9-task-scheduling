cmake_minimum_required(VERSION 3.16)
project(ip9_task_scheduling)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/ros/jazzy")

enable_testing()

include(FetchContent)

FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
)

FetchContent_Declare(
        BehaviorTree
        GIT_REPOSITORY https://github.com/BehaviorTree/BehaviorTree.CPP.git
        GIT_TAG 4.7.3
)

FetchContent_MakeAvailable(googletest BehaviorTree)

find_package(sdformat15 REQUIRED)
find_package(gz-math8 REQUIRED)
find_package(gz-transport13 REQUIRED)

# ROS2
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Qt5 COMPONENTS
        Core
        Gui
        Widgets
        Charts
        Sql
        REQUIRED)

add_executable(ip9_task_scheduling
        main.cpp
        model/event.h
        model/event.cpp
        model/simulation.h
        model/robot.h
        model/robot.cpp
        model/robot_state.h
        model/robot_state.cpp
        model/context.h

        view/term.h
        observer/observer.h

        # util/log.h
        # util/db.h
        util/types.h

        # view/timeline_view.h
        # view/event_plan.h

        # behaviour/nodes/nav_to_point.h
        # behaviour/dummy.h
        # behaviour/nodes/event_handler.h
        # behaviour/nodes/tour.h
        # behaviour/nodes/robot_state.h

        sim/gz_lib.h
        sim/ros/path_node.h
        sim/ros/marker.h

)

set_target_properties(ip9_task_scheduling PROPERTIES
        AUTOMOC ON
        AUTORCC ON
        AUTOUIC ON
)

target_link_libraries(ip9_task_scheduling
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::Charts
        Qt5::Sql
        sdformat15::sdformat15
        gz-math8::gz-math8
        gz-transport13::gz-transport13
        gz-msgs10::gz-msgs10
        BT::behaviortree_cpp
        rclcpp::rclcpp
        tf2_ros::tf2_ros
        ${nav2_msgs_TARGETS}
        ${visualization_msgs_TARGETS}
        ${tf2_geometry_msgs_TARGETS}
)

get_target_property(QT_INCLUDES Qt5::Widgets INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(ip9_task_scheduling PRIVATE ${QT_INCLUDES})


add_executable(transform_node
    sim/ros/transform_node.cpp
)

target_link_libraries(transform_node
    rclcpp::rclcpp
    tf2_ros::tf2_ros
    gz-math8::gz-math8
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    ${nav2_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)
