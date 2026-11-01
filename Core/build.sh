# Source ROS Enviroment
source install/conan/conanrosenv.sh

# Build
colcon build --base-paths src/functional/interfaces/* src/hardware/interfaces/*

source install/setup.sh

colcon build \
--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
--paths src/middleware/* src/hardware/* \
src/functional/control/* src/functional/perception/*

# Build an individual package
# colcon build --packages-select peripheral --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON