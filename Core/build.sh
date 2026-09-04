# Source ROS Enviroment
# source install/conan/conanrosenv.sh

# Build
colcon build --paths src/msg/* --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++
source install/setup.sh
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++

# Build an individual package
# colcon build --packages-select board_controller