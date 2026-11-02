# Source ROS Enviroment
source install/conan/conanrosenv.sh

# Build
colcon build \
--base-paths src/application/interfaces \
             src/functional/interfaces \
             src/hardware/interfaces

source install/setup.sh

colcon build \
--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
--paths src/application/* \
        src/functional/control/* src/functional/perception/* \
        src/hardware/* \
--base-paths  src/middleware

# Build an individual package
# colcon build --packages-select peripheral --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON