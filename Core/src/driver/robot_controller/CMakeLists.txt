cmake_minimum_required(VERSION 3.20)
project(robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find dependencies ##
# ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
# Host
find_package(board_controller_msg REQUIRED)
find_package(robot_controller_msg REQUIRED)

## Launches ##
# Install all launch files
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

## Targets ##
# Robot Controller
file(GLOB_RECURSE robot_controller_node_src 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_controller_node/*.cpp
)

add_executable(robot_controller ${robot_controller_node_src})

target_include_directories(robot_controller PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_controller_node
    ${CMAKE_CURRENT_SOURCE_DIR}/include/peripheral
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utility
)

target_link_libraries(robot_controller PUBLIC
    # ROS
    rclcpp::rclcpp
    std_srvs::std_srvs
    # ArmBot
    board_controller_msg::board_controller_msg
    robot_controller_msg::robot_controller_msg
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
