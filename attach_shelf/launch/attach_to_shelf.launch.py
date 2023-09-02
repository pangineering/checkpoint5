import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'obstacle',
            default_value='0.3',
            description='Obstacle distance in meters'
        ),
        DeclareLaunchArgument(
            'degrees',
            default_value='-90',
            description='Rotation angle in degrees'
        ),
        DeclareLaunchArgument(
            'final_approach',
            default_value='false',
            description='Whether to initiate final approach'
        ),

        Node(
            package="attach_shelf",  # Replace with the actual package name
            executable="approach_service_server",  # Replace with the executable name
            name="approach_service_server"
        ),

        Node(
            package="attach_shelf",  # Replace with the actual package name
            executable="pre_approach_v2",  # Replace with the executable name
            name="pre_approach_v2",
            parameters=[
                {"obstacle_distance": LaunchConfiguration('obstacle')},
                {"rotation_angle": LaunchConfiguration('degrees')},
                {"initiate_final_approach": LaunchConfiguration('final_approach')}
            ]
        ),
    ])
