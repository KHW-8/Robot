from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    peripheral = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("peripheral"),
            "launch",
            "peripheral.launch.py"
        ])
    )

    vision = Node(
        package="vision",
        executable="vision",
        output="screen"
    )

    robot_controller = Node(
        package="robot",
        executable="robot_controller",
        output="screen"
    )

    return LaunchDescription([
        peripheral,
        vision,
        robot_controller
    ])