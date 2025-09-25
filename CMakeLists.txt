cmake_minimum_required(VERSION 4.0)
project(ip9_task_scheduling)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)


find_package(Qt5 COMPONENTS
        Core
        Gui
        Widgets
        REQUIRED)

add_executable(ip9_task_scheduling src/main.cpp
        include/Timeline.h
        include/Task.h
        include/MainView.h)
target_link_libraries(ip9_task_scheduling
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
)

