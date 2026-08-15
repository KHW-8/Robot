# Source Conan enviroment
source install/conan/conanrosenv.sh

# Source ROS enviroment
source install/setup.sh

ros2 launch example led.launch.py
# ros2 launch board_controller board_controller.launch.py