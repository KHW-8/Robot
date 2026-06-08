from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ros_robot_controller_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("ros_robot_controller"),
            "launch",
            "ros_robot_controller.launch.py"
        ])
    )

    led_node = Node(
        package="example",
        executable="led",
        output="screen"
    )

    return LaunchDescription([
        ros_robot_controller_launch,
        led_node
    ])