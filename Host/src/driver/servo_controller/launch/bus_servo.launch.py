from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    arm_controller_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("ros_robot_controller"),
            "launch",
            "ros_robot_controller.launch.py"
        ])
    )

    bus_servo_node = Node(
        package="servo_controller",
        executable="bus_servo",
        output="screen"
    )

    return LaunchDescription([
        arm_controller_launch,
        bus_servo_node
    ])