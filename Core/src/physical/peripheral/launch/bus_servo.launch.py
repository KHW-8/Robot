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

    bus_servo = Node(
        package="peripheral",
        executable="bus_servo",
        output="screen"
    )

    return LaunchDescription([
        board_controller,
        bus_servo
    ])