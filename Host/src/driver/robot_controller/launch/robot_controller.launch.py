from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robot_controller_node = Node(
        package="robot_controller",
        executable="robot_controller",
        output="screen"
    )

    return LaunchDescription([
        robot_controller_node
    ])