cmake_minimum_required(VERSION 3.20)
project(board)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find dependencies ##
# Boost
find_package(Boost CONFIG REQUIRED)
# Core
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

## Targets ##
#  Board Controller
file(GLOB_RECURSE board_controller_src 
    ${CMAKE_SOURCE_DIR}/src/board_controller/*.cpp
)

add_executable(board_controller 
    ${board_controller_src}
)

target_include_directories(board_controller PRIVATE
    ${CMAKE_SOURCE_DIR}/include/board_controller
    ${CMAKE_SOURCE_DIR}/../../global/include/utility
)

target_link_libraries(board_controller PUBLIC
    # Boost
    Boost::boost
    # Core
    board_msg::board_msg
    # ROS
    rclcpp::rclcpp
    std_srvs::std_srvs
    # SymEngine
    symengine
)

install(TARGETS
    board_controller
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
