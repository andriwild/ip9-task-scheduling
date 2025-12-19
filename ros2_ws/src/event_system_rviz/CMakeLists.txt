cmake_minimum_required(VERSION 3.8)
project(event_system_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(event_system_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/event_system_rviz/des_panel.hpp
)

add_library(des_panel src/des_panel.cpp ${MOC_FILES})
target_include_directories(des_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(des_panel
  pluginlib
  rviz_common
event_system_interfaces
)
install(TARGETS des_panel
        EXPORT export_event_system_rviz
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
install(FILES icons/classes/DesPanel.png
        DESTINATION share/${PROJECT_NAME}/icons/classes
)
ament_export_include_directories(include)
ament_export_targets(export_event_system_rviz)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_package()
