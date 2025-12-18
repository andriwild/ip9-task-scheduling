cmake_minimum_required(VERSION 3.8)
project(des_panel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(des_msgs REQUIRED)


set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/des_panel/des_panel.hpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(des_panel src/des_panel.cpp ${MOC_FILES})
target_include_directories(des_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(des_panel
  pluginlib
  rviz_common
  des_msgs
)

install(TARGETS des_panel
        EXPORT export_rviz_des_panel
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)

install(FILES icons/classes/DesPanel.png
        DESTINATION share/${PROJECT_NAME}/icons/classes/
)


ament_export_include_directories(include)
ament_export_targets(export_rviz_des_panel)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)


ament_package()
