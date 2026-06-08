from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    arm_controller_node = Node(
        package="arm_controller",
        executable="arm_controller",
        output="screen"
    )

    return LaunchDescription([
        arm_controller_node
    ])