cmake_minimum_required(VERSION 3.5)
project(nav2_rtk_spline_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

set(library_name nav2_rtk_spline_planner)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_util
  nav2_core
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  nav2_costmap_2d
  pluginlib
)

add_library(${library_name} SHARED
  src/rtk_spline_planner.cpp
  src/cubic_spline.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_compile_features(${library_name} PUBLIC cxx_std_17)

pluginlib_export_plugin_description_file(nav2_core rtk_spline_planner_plugin.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES rtk_spline_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
