# Source ROS Enviroment
source install/conan/conanrosenv.sh

# Build
colcon build --base-paths src/hardware/msg/*

source install/setup.sh

colcon build \
--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
--paths src/middleware/* src/control/* src/hardware/*

# Build an individual package
# colcon build --packages-select peripheral --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON