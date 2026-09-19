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

    kinematics_calculator = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("kinematics"),
            "launch",
            "kinematics_calculator.launch.py"
        ])
    )

    robot_controller = Node(
        package="robot",
        executable="robot_controller",
        output="screen"
    )

    return LaunchDescription([
        board_controller,
        kinematics_calculator,
        robot_controller
    ])