cmake_minimum_required(VERSION 3.8)
project(event_system_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(event_system_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 COMPONENTS Charts REQUIRED)

set(CMAKE_AUTOMOC ON)

qt5_wrap_cpp(MOC_FILES
  include/event_system_rviz/des_panel.hpp
  include/event_system_rviz/des_system_config.hpp
  include/event_system_rviz/des_timeline_panel.hpp
  include/event_system_rviz/des_metrics_panel.hpp
  include/event_system_rviz/timeline/timeline.hpp
)

add_library(event_system_rviz
  src/des_panel.cpp
  src/des_system_config.cpp
  src/des_timeline_panel.cpp
  src/des_metrics_panel.cpp
  ${MOC_FILES}
)

target_include_directories(event_system_rviz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(event_system_rviz
  pluginlib
  rviz_common
  event_system_msgs
)
target_link_libraries(event_system_rviz Qt5::Charts)

install(TARGETS event_system_rviz
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
install(FILES
    icons/classes/DesPanel.png
    icons/classes/DesTimelinePanel.png
    icons/classes/DesMetricsPanel.png
    icons/classes/DesSystemConfig.png
    DESTINATION share/${PROJECT_NAME}/icons/classes
)
ament_export_include_directories(include)
ament_export_targets(export_event_system_rviz)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_package()
