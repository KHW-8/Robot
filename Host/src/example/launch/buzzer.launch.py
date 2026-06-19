from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_controller_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("robot_controller"),
            "launch",
            "robot_controller.launch.py"
        ])
    )

    buzzer_node = Node(
        package="example",
        executable="buzzer",
        output="screen"
    )

    return LaunchDescription([
        robot_controller_launch,
        buzzer_node
    ])