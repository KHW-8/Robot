from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera = Node(
        package="peripheral",
        executable="camera",
        output="screen"
    )

    return LaunchDescription([
        camera
    ])