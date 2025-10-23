cmake_minimum_required(VERSION 3.16)
project(ip9_task_scheduling)
set(CMAKE_CXX_STANDARD 20)

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
        GIT_TAG 4.8.0
)
FetchContent_MakeAvailable(BehaviorTree)

find_package(Qt5 COMPONENTS
        Core
        Gui
        Widgets
        Charts
        Sql
        REQUIRED)

add_executable(ip9_task_scheduling
        main.cpp
        view/timeline_view.h
        view/map_view.h
        model/event.h
        model/robot.h
        algo/rnd.h
        util/util.h
        util/log.h
        util/db.h
        algo/algo.h
        datastructure/graph.h
        model/simulation.h
        view/robot_item.h
        model/planner.h
        view/helper.h
        datastructure/tree.h
        view/event_plan.h
        util/tree_composer.h
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
        BT::behaviortree_cpp
)

add_executable(tests
        tests/test_graph.cpp
        tests/test_tree.cpp
)

target_link_libraries(tests
        gtest
        gtest_main
)

include(GoogleTest)
gtest_discover_tests(tests)