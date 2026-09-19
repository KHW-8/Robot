cmake_minimum_required(VERSION 3.20)
project(kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find dependencies ##
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(symengine CONFIG REQUIRED)

## Launches ##
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

## Targets ##
# Kinematics Calculator
file(GLOB_RECURSE kinematics_calculator_src
    ${CMAKE_SOURCE_DIR}/src/kinematics_calculator/*.cpp
)

add_executable(kinematics_calculator ${kinematics_calculator_src})

target_include_directories(kinematics_calculator PRIVATE
    ${CMAKE_SOURCE_DIR}/include/kinematics_calculator
    ${CMAKE_SOURCE_DIR}/include/kinematics_calculator/DH_parameter
    ${CMAKE_SOURCE_DIR}/include/kinematics_calculator/math
    ${CMAKE_SOURCE_DIR}/include/kinematics_calculator/kinematics/forward_kinematics
    ${CMAKE_SOURCE_DIR}/../global/include/math
)

target_link_libraries(kinematics_calculator PRIVATE
    # ROS
    rclcpp::rclcpp
    std_srvs::std_srvs
    # SymEngine
    symengine
)

# Install all targets
install(TARGETS
    kinematics_calculator
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
