from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    topic = LaunchConfiguration('topic')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'topic',
                default_value='/scara_arm/tester_ui',
                description='Topic name to publish TesterUICommand messages to.',
            ),
            SetEnvironmentVariable(name='TESTER_UI_TOPIC', value=topic),
            Node(
                package='robot_ui',
                executable='tester_ui',
                name='tester_ui',
                output='screen',
            ),
            Node(
                package='robot_controller',
                executable='controller',
                name='robot_controller',
                output='screen',
            ),
        ]
    )
