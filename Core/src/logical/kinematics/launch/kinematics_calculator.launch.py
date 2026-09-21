from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_controller = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("robot"),
            "launch",
            "robot_controller.launch.py"
        ])
    )

    kinematics_calculator = Node(
        package="kinematics",
        executable="kinematics_calculator",
        output="screen"
    )

    return LaunchDescription([
        robot_controller,
        kinematics_calculator
    ])