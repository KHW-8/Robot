# Source ROS Enviroment
source install/conan/conanrosenv.sh

# Build
colcon build --paths src/msg/* --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.sh
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Build an individual package
# colcon build --packages-select robot_controller