cmake_minimum_required(VERSION 3.20)
project(robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find dependencies ##
# Core
find_package(board_msg REQUIRED)
find_package(peripheral_msg REQUIRED)
# ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
# SymEngine
find_package(symengine REQUIRED)

## Launches ##
# Install all launch files
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

## Targets ##
# Robot Controller
file(GLOB_RECURSE robot_controller_src
    ${CMAKE_SOURCE_DIR}/src/robot_controller/*.cpp
)

add_executable(robot_controller ${robot_controller_src})

target_include_directories(robot_controller PRIVATE
    ${CMAKE_SOURCE_DIR}/include/robot_controller
    ${CMAKE_SOURCE_DIR}/../../global/include/utility
    ${CMAKE_SOURCE_DIR}/../../physical/peripheral/include/bus_servo
)

target_link_libraries(robot_controller PRIVATE
    # ROS
    rclcpp::rclcpp
    std_srvs::std_srvs
    # ArmBot
    board_msg::board_msg
    peripheral_msg::peripheral_msg
    # SymEngine
    symengine
)

# Install all targets
install(TARGETS
    robot_controller
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
