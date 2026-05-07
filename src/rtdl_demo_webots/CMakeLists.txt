cmake_minimum_required(VERSION 3.8)
project(rtdl_demo_webots)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rtdl_demo_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(rtdl_demo_webots_driver SHARED
  src/CarDriver.cpp
  src/WorldBridgeDriver.cpp
)

target_include_directories(rtdl_demo_webots_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rtdl_demo_webots_driver
  yaml-cpp
  rclcpp
  geometry_msgs
  webots_ros2_driver
  pluginlib
  rtdl_demo_interfaces
)

pluginlib_export_plugin_description_file(webots_ros2_driver rtdl_demo_webots_driver.xml)

install(TARGETS
  rtdl_demo_webots_driver
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch worlds resource config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
