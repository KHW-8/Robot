cmake_minimum_required(VERSION 3.20)
project(peripheral)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find dependencies ##
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
# Host
find_package(peripheral_msg REQUIRED)

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
)

target_link_libraries(bus_servo PUBLIC
    rclcpp::rclcpp
    std_srvs::std_srvs
    peripheral_msg::peripheral_msg
)

# Buzzer #
add_executable(buzzer src/buzzer/buzzer.cpp)

target_include_directories(buzzer PUBLIC
    ${CMAKE_SOURCE_DIR}/include/buzzer
)

target_link_libraries(buzzer PUBLIC
    rclcpp::rclcpp
    std_srvs::std_srvs
    peripheral_msg::peripheral_msg
)

# LED #
add_executable(led src/led/led.cpp)

target_include_directories(led PUBLIC
    ${CMAKE_SOURCE_DIR}/include/led
)

target_link_libraries(led PUBLIC
    rclcpp::rclcpp
    std_srvs::std_srvs
    peripheral_msg::peripheral_msg
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
