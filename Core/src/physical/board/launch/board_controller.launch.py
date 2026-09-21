from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    BoardController = Node(
        package="board",
        executable="board_controller",
        output="screen"
    )

    return LaunchDescription([
        BoardController
    ])