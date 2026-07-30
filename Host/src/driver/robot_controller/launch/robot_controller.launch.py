from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    board_controller_node = Node(
        package="board_controller",
        executable="board_controller",
        output="screen"
    )

    robot_controller_node = Node(
        package="robot_controller",
        executable="robot_controller",
        output="screen"
    )

    return LaunchDescription([
        board_controller_node,
        robot_controller_node
    ])