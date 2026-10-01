cmake_minimum_required(VERSION 3.20)
project(peripheral)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find dependencies ##
# Core
find_package(peripheral_msg REQUIRED)
find_package(board_msg REQUIRED)
# ROS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
# SymEngine
find_package(symengine REQUIRED)

## Launch files ##
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

## Executables ##
# Bus Servo Node #
add_executable(bus_servo src/bus_servo/bus_servo.cpp)

target_include_directories(bus_servo PUBLIC
    ${CMAKE_SOURCE_DIR}/include/bus_servo
    ${CMAKE_SOURCE_DIR}/include//peripheral
    ${CMAKE_SOURCE_DIR}/../../global/include/utility
)

target_link_libraries(bus_servo PUBLIC
    # Core
    peripheral_msg::peripheral_msg
    board_msg::board_msg
    # ROS
    rclcpp::rclcpp
    std_srvs::std_srvs
    # SymEngine
    symengine
)

# Buzzer #
add_executable(buzzer src/buzzer/buzzer.cpp)

target_include_directories(buzzer PUBLIC
    ${CMAKE_SOURCE_DIR}/include/buzzer
    ${CMAKE_SOURCE_DIR}/include//peripheral
    ${CMAKE_SOURCE_DIR}/../../global/include/utility
)

target_link_libraries(buzzer PUBLIC
    # Core
    peripheral_msg::peripheral_msg
    board_msg::board_msg
    # ROS
    rclcpp::rclcpp
    std_srvs::std_srvs
    # SymEngine
    symengine
)

# LED #
add_executable(led src/led/led.cpp)

target_include_directories(led PUBLIC
    ${CMAKE_SOURCE_DIR}/include/led
    ${CMAKE_SOURCE_DIR}/include//peripheral
    ${CMAKE_SOURCE_DIR}/../../global/include/utility
)

target_link_libraries(led PUBLIC
    # Core
    peripheral_msg::peripheral_msg
    board_msg::board_msg
    # ROS
    rclcpp::rclcpp
    std_srvs::std_srvs
    # SymEngine
    symengine
)

install(TARGETS
    bus_servo
    buzzer
    led
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
