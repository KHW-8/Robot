# Install ROS packages
# sudo apt install ros2-$ROS_DISTRO-cv-bridge

# Install needed Python packages
pip install conan catkin_pkg empy lark numpy

# Install source code dependencies by Conan
conan install dependency/conanfile.py --build=missing --output-folder install/conan