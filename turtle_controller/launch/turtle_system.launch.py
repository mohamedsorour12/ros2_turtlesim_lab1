from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare configurable launch arguments for speeds
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='2.0',
        description='Maximum linear speed for the turtle'
    )
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='1.5',
        description='Maximum angular speed for the turtle'
    )

    # 1. Turtlesim simulator node
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen'
    )

    # 2. Controller node (goToPose Action Server & Speed Parameter Service)
    controller_node = Node(
        package='turtle_controller',
        executable='controller_node',
        name='turtle_controller_node',
        parameters=[{
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
        }],
        output='screen'
    )

    # 3. Reset service node
    reset_service_node = Node(
        package='turtle_controller',
        executable='reset_service_node',
        name='reset_service_node',
        output='screen'
    )

    # 4. Square action server node
    square_action_server_node = Node(
        package='turtle_controller',
        executable='square_action_server_node',
        name='square_action_server_node',
        output='screen'
    )

    return LaunchDescription([
        max_linear_speed_arg,
        max_angular_speed_arg,
        turtlesim_node,
        controller_node,
        reset_service_node,
        square_action_server_node,
    ])
