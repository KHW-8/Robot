from launch import LaunchDescription
from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    kinematics_calculator = Node(
        package="kinematics",
        executable="kinematics_calculator",
        output="screen"
    )

    return LaunchDescription([
        kinematics_calculator
    ])