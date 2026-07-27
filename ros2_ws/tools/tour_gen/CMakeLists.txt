cmake_minimum_required(VERSION 3.16)
project(tour_gen CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(tour_gen main.cpp)
target_compile_definitions(tour_gen PRIVATE NDEBUG)
target_include_directories(tour_gen SYSTEM PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/vislib2/VisLib
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(tour_gen ${OpenCV_LIBS} gmp gmpxx)
