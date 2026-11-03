cmake_minimum_required(VERSION 3.20)
project(peripheral)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find dependencies ##
# Core
find_package(peripheral_msg REQUIRED)
find_package(board_msg REQUIRED)
# OpenCV
find_package(OpenCV REQUIRED COMPONENTS core)
# ROS
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

## Launch files ##
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

## Targets ##
# Bus Servo #
add_executable(bus_servo src/bus_servo/bus_servo.cpp)

target_include_directories(bus_servo PUBLIC
    ${CMAKE_SOURCE_DIR}/include/bus_servo
    ${CMAKE_SOURCE_DIR}/include/peripheral
    ${CMAKE_SOURCE_DIR}/include/global
)

target_link_libraries(bus_servo PUBLIC
    # Core
    peripheral_msg::peripheral_msg
    board_msg::board_msg
    # ROS
    rclcpp::rclcpp
    std_srvs::std_srvs
)

# Buzzer #
add_executable(buzzer src/buzzer/buzzer.cpp)

target_include_directories(buzzer PUBLIC
    ${CMAKE_SOURCE_DIR}/include/buzzer
    ${CMAKE_SOURCE_DIR}/include/peripheral
    ${CMAKE_SOURCE_DIR}/include/global
)

target_link_libraries(buzzer PUBLIC
    # Core
    peripheral_msg::peripheral_msg
    board_msg::board_msg
    # ROS
    rclcpp::rclcpp
    std_srvs::std_srvs
)

# Camera 
add_executable(camera src/camera/camera.cpp)

target_include_directories(camera PUBLIC
    ${CMAKE_SOURCE_DIR}/include/camera
    ${CMAKE_SOURCE_DIR}/include/peripheral
    ${CMAKE_SOURCE_DIR}/../../3rdparty/sdk/depth_camera/deptrum/aurora900/include    
)

target_link_libraries(camera PUBLIC
    # Core
    peripheral_msg::peripheral_msg
    # Deptrum
    ${CMAKE_SOURCE_DIR}/../../3rdparty/sdk/depth_camera/deptrum/aurora900/lib/libdeptrum_stream_aurora900.so.1.1.19
    # OpenCV
    opencv_core
    # ROS
    cv_bridge::cv_bridge
    rclcpp::rclcpp
    std_srvs::std_srvs
    sensor_msgs::sensor_msgs
)

set_target_properties(camera PROPERTIES
   INSTALL_RPATH $ORIGIN
)

# LED #
add_executable(led src/led/led.cpp)

target_include_directories(led PUBLIC
    ${CMAKE_SOURCE_DIR}/include/led
    ${CMAKE_SOURCE_DIR}/include//peripheral
    ${CMAKE_SOURCE_DIR}/include/global
)

target_link_libraries(led PUBLIC
    # Core
    peripheral_msg::peripheral_msg
    board_msg::board_msg
    # ROS
    rclcpp::rclcpp
    std_srvs::std_srvs
)

## Install all targets ##
install(TARGETS
    bus_servo
    buzzer
    camera
    led
    DESTINATION lib/${PROJECT_NAME}
)

install(FILES
   ${CMAKE_SOURCE_DIR}/../../3rdparty/sdk/depth_camera/deptrum/aurora900/lib/libdeptrum_stream_aurora900.so.1.1.19
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
