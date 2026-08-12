# The simulation itself, free of ROS. Buildable on its own:
#
#   cmake -S src/event_system_core/src -B build-headless
#   cmake --build build-headless
#
# The ROS package pulls this in via add_subdirectory and adds the ROS layer on top.

cmake_minimum_required(VERSION 3.16)
project(des_core CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# BehaviorTree.CPP is a standalone package, but the copy installed here comes from
# ROS and pulls ament_cmake_core in, which in turn needs the ament_package Python
# module. Find that prefix ourselves so an IDE needs no extra settings. Only when
# we are the top-level project: the ROS package build brings its own environment.
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR AND NOT EXISTS "/usr/lib/cmake/behaviortree_cpp")
  file(GLOB ros_prefixes "/opt/ros/*")
  foreach(prefix ${ros_prefixes})
    if(EXISTS "${prefix}/share/behaviortree_cpp")
      list(APPEND CMAKE_PREFIX_PATH "${prefix}")
      file(GLOB ament_py "${prefix}/lib/python*/site-packages")
      list(GET ament_py 0 ament_py)
      set(ENV{PYTHONPATH} "${ament_py}:$ENV{PYTHONPATH}")
      break()
    endif()
  endforeach()
endif()

find_package(nlohmann_json REQUIRED)
find_package(behaviortree_cpp REQUIRED)

set(DES_SRC "${CMAKE_CURRENT_SOURCE_DIR}")
get_filename_component(DES_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../config" ABSOLUTE)

file(GLOB_RECURSE DES_SOURCES CONFIGURE_DEPENDS "${DES_SRC}/*.cpp")
file(GLOB_RECURSE DES_HEADERS CONFIGURE_DEPENDS "${DES_SRC}/*.h" "${DES_SRC}/*.hpp")

# An IDE may put its build tree next to the sources, and CMake generates a .cpp
# of its own in there. Keep the glob to the sources.
foreach(list_var DES_SOURCES DES_HEADERS)
  list(FILTER ${list_var} EXCLUDE REGEX "^${CMAKE_BINARY_DIR}/")
  list(FILTER ${list_var} EXCLUDE REGEX "/(cmake-build|build)[^/]*/")
endforeach()

# Everything that talks to ROS lives in the frontends, not in the library.
# Listing those files here would only give an IDE broken include paths.
list(REMOVE_ITEM DES_SOURCES
  "${DES_SRC}/main.cpp"
  "${DES_SRC}/main_headless.cpp"
  "${DES_SRC}/runner/impl/sim_runner.cpp"
)
list(REMOVE_ITEM DES_HEADERS
  "${DES_SRC}/observer/ros.h"
  "${DES_SRC}/sim/ros/config.h"
  "${DES_SRC}/sim/ros/controller.h"
  "${DES_SRC}/sim/ros/path_node.h"
  "${DES_SRC}/runner/impl/ros_runner.h"
  "${DES_SRC}/runner/impl/sim_runner.h"
  "${DES_SRC}/runner/impl/snapshot_builder.h"
)

add_library(des_core STATIC ${DES_SOURCES} ${DES_HEADERS})
target_include_directories(des_core PUBLIC ${DES_SRC})
target_compile_definitions(des_core PUBLIC DES_CONFIG_DIR="${DES_CONFIG_DIR}")
target_link_libraries(des_core PUBLIC
  nlohmann_json::nlohmann_json
  behaviortree_cpp::behaviortree_cpp
  atomic
)

add_executable(des_headless "${DES_SRC}/main_headless.cpp")
target_link_libraries(des_headless des_core)

# The ROS build of BehaviorTree.CPP needs libament_index_cpp.so alongside it, and
# a DT_RUNPATH does not apply to a dependency's own dependencies. DT_RPATH does,
# so the binary runs without LD_LIBRARY_PATH.
target_link_options(des_headless PRIVATE "LINKER:--disable-new-dtags")
