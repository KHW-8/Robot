cmake_minimum_required(VERSION 3.20)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find dependencies ##
# ROS
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
# OpenCV
find_package(OpenCV REQUIRED COMPONENTS core highgui)
# YAML-CPP
find_package(yaml-cpp REQUIRED)


## Launches ##
# Install all launch files
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

## Targets ##
add_executable(vision src/vision/vision.cpp)

target_include_directories(vision PUBLIC
    ${CMAKE_SOURCE_DIR}/include/vision
)

target_link_libraries(vision PUBLIC
    # OpenCV
    opencv_core
    opencv_highgui
    # ROS
    cv_bridge::cv_bridge
    rclcpp::rclcpp
    std_srvs::std_srvs
    sensor_msgs::sensor_msgs
    # YAML-CPP
    yaml-cpp::yaml-cpp
)

## Install ##
install(TARGETS
    vision
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
