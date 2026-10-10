from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackage

def generate_launch_description():
    camera = Node(
        package="peripheral",
        executable="camera",
        output="screen"
    )

    return LaunchDescription([
        camera
    ])