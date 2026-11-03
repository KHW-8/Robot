from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    control_panel = Node(
        package="gui",
        executable="control_panel",
        output="screen",
    )

    return LaunchDescription([
        control_panel,
    ])