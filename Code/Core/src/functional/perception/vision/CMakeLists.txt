cmake_minimum_required(VERSION 3.20)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find dependencies ##
# Core
find_package(kinematics_msgs REQUIRED)
# ROS
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
# OpenCV
find_package(OpenCV REQUIRED COMPONENTS core highgui)
# SymEngine
find_package(symengine REQUIRED)
# YAML-CPP
find_package(yaml-cpp REQUIRED)


## Launches ##
# Install all launch files
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

## Targets ##
file(GLOB_RECURSE vision_src
    ${CMAKE_SOURCE_DIR}/src/vision/*.cpp
    ${CMAKE_SOURCE_DIR}/../../global/src/math/*.cpp
)


add_executable(vision ${vision_src})

target_include_directories(vision PUBLIC
    ${CMAKE_SOURCE_DIR}/include/vision
    ${CMAKE_SOURCE_DIR}/../../global/include/math
)

target_link_libraries(vision PUBLIC
    # Core
    kinematics_msgs::kinematics_msgs
    # OpenCV
    opencv_core
    opencv_highgui
    # ROS
    cv_bridge::cv_bridge
    rclcpp::rclcpp
    std_srvs::std_srvs
    sensor_msgs::sensor_msgs
    # SymEngine
    symengine
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
