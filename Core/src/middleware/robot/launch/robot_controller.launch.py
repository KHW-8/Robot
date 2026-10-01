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

    bus_servo = Node(
        package="peripheral",
        executable="bus_servo",
        output="screen"
    )


    buzzer = Node(
        package="peripheral",
        executable="buzzer",
        output="screen"
    )


    led = Node(
        package="peripheral",
        executable="led",
        output="screen"
    )

    robot_controller = Node(
        package="robot",
        executable="robot_controller",
        output="screen"
    )

    return LaunchDescription([
        board_controller,
        bus_servo,
        buzzer,
        led,
        robot_controller
    ])