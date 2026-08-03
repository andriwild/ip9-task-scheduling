cmake_minimum_required(VERSION 3.16)
project(tour_gen CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(tour_gen main.cpp)
target_compile_definitions(tour_gen PRIVATE NDEBUG)
target_compile_options(tour_gen PRIVATE -ffp-contract=off)
target_include_directories(tour_gen SYSTEM PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/vislib2/VisLib
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(tour_gen ${OpenCV_LIBS} nlohmann_json::nlohmann_json gmp gmpxx)

add_executable(tsp_test test.cpp)
target_compile_options(tsp_test PRIVATE -ffp-contract=off)
target_include_directories(tsp_test SYSTEM PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/vislib2/VisLib
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(tsp_test ${OpenCV_LIBS} gmp gmpxx)
