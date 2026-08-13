# Source ROS Enviroment
source install/conan/conanrosenv.sh

# Build
colcon build --paths src/msg/*
source install/setup.sh
colcon build

# Build an individual package
# colcon build --packages-select robot_controller