# Install all dependencies by Conan
conan profile detect --force
conan install dependency/conanfile.py --build=missing --output-folder install/conan --deployer=full_deploy

# Source
source install/conan/conanrosenv.sh

# Build
colcon build --paths src/msg
source install/setup.sh
colcon build

# Build an individual package
# colcon build --packages-select robot_controller