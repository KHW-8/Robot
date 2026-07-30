cmake_minimum_required(VERSION 3.8)
project(board_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## find dependencies ##
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robot_controller_msg REQUIRED)

# Includes
set(third_party_path ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty)
set(include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/board_controller
    # Third party
    ${third_party_path}/serial/include
)

include_directories(${include})

## Third party sources ##
set(third_party_src)
if(UNIX)
    list(APPEND third_party_src ${third_party_path}/serial/src/serial.cc)
    list(APPEND third_party_src ${third_party_path}/serial/src/impl/unix.cc)
    list(APPEND third_party_src ${third_party_path}/serial/src/impl/list_ports/list_ports_linux.cc)
elseif(WIN32)
    list(APPEND third_party_src ${third_party_path}/serial/src/serial.cc)
    list(APPEND third_party_src ${third_party_path}/serial/src/impl/win.cc)
    list(APPEND third_party_src ${third_party_path}/serial/src/impl/list_ports/list_ports_win.cc)
endif()
foreach(s IN LISTS third_party_src)
    message(${s})
endforeach()

## Launch files ##
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

## Executables ##
add_executable(board_controller 
    src/board_controller_node.cpp
    ${third_party_src}
)
ament_target_dependencies(board_controller 
    rclcpp 
    std_srvs 
    robot_controller_msg
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
