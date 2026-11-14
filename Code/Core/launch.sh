# Source Conan enviroment
source install/conan/conanrosenv.sh

# Source ROS enviroment
source install/setup.sh

# ros2 launch robot robot_controller.launch.py
ros2 launch vision vision.launch.py
# ros2 launch kinematics kinematics_calculator.launch.py