from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    board_controller = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("board"),
            "launch",
            "board_controller.launch.py"
        ])
    )

    buzzer = Node(
        package="peripheral",
        executable="buzzer",
        output="screen"
    )

    return LaunchDescription([
        board_controller,
        buzzer
    ])