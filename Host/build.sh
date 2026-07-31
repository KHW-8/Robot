# Install all dependencies by Conan
conan install . --build=missing --output-folder install/conan --deployer=full_deploy

# Source
source install/conan/conanrosenv.sh

# Build
colcon build --paths src/msg/*_msg*
source install/setup.sh
colcon build

# colcon build --packages-select board_controller_msg