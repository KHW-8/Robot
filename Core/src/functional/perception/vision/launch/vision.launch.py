from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    camera = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("peripheral"),
            "launch",
            "camera.launch.py"
        ])
    )

    vision = Node(
        package="vision",
        executable="vision",
        output="screen"
    )

    return LaunchDescription([
        camera,
        vision
    ])