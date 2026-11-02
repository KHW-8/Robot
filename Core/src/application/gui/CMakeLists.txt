cmake_minimum_required(VERSION 3.20)
project(gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find dependencies ##
# Core
find_package(gui_msgs REQUIRED)
# ROS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

## Launch files ##
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

## Targets ##
file(GLOB_RECURSE control_panel_src
    ${CMAKE_SOURCE_DIR}/src/control_panel/*.cpp
)

add_executable(control_panel
    ${control_panel_src}
)

target_include_directories(control_panel PRIVATE
    ${CMAKE_SOURCE_DIR}/include/control_panel
)

target_link_libraries(control_panel PUBLIC
    # Core
    gui_msgs::gui_msgs
    # ROS
    rclcpp::rclcpp
    std_srvs::std_srvs
)

## Install ##
install(TARGETS
    control_panel
    DESTINATION lib/${PROJECT_NAME}
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
