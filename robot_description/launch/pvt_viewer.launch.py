from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description() -> LaunchDescription:
    desc_share = get_package_share_directory('robot_description')
    display_launch = os.path.join(desc_share, 'launch', 'display.launch.py')

    rviz_config_default = os.path.join(desc_share, 'rviz', 'robot_description.rviz')

    endpoint = LaunchConfiguration('endpoint')
    mode = LaunchConfiguration('mode')
    conflate = LaunchConfiguration('conflate')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    save_csv = LaunchConfiguration('save_csv')

    joint_states_mapper_topic = LaunchConfiguration('joint_states_mapper_topic')
    joint_states_topic = LaunchConfiguration('joint_states_topic')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'endpoint',
                default_value='ipc:///tmp/motor_cmd',
                description='ZMQ endpoint for incoming PVT points.',
            ),
            DeclareLaunchArgument(
                'mode',
                default_value='SUB',
                description='ZMQ mode: SUB (PUB/SUB) or PULL (PUSH/PULL).',
            ),
            DeclareLaunchArgument(
                'conflate',
                default_value='true',
                description='If true, only keep the latest ZMQ message (reduces lag).',
            ),
            DeclareLaunchArgument(
                'publish_rate_hz',
                default_value='50.0',
                description='ROS publish rate for JointState (Hz).',
            ),
            DeclareLaunchArgument(
                'save_csv',
                default_value='',
                description='Optional CSV log path (empty disables).',
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=rviz_config_default,
                description='Path to an RViz config (.rviz).',
            ),
            DeclareLaunchArgument(
                'joint_states_mapper_topic',
                default_value='/joint_states_mapper',
                description='Topic where pvt_to_joint publishes raw/cumulative JointState (input to joint_state_mapper).',
            ),
            DeclareLaunchArgument(
                'joint_states_topic',
                default_value='/joint_states',
                description='Mapped JointState output topic (consumed by robot_state_publisher).',
            ),

            # RViz + robot_state_publisher + joint_state_mapper (unchanged)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(display_launch),
                launch_arguments={
                    'rviz_config': rviz_config,
                    'joint_states_mapper_topic': joint_states_mapper_topic,
                    'joint_states_topic': joint_states_topic,
                }.items(),
            ),

            # ZMQ -> JointState publisher (without conflate)
            Node(
                package='robot_description',
                executable='pvt_to_joint',
                name='pvt_to_joint',
                output='screen',
                condition=UnlessCondition(conflate),
                arguments=[
                    '--endpoint', endpoint,
                    '--mode', mode,
                    '--topic', joint_states_mapper_topic,
                    '--rate', publish_rate_hz,
                    '--save', save_csv,
                ],
            ),

            # ZMQ -> JointState publisher (with conflate flag)
            Node(
                package='robot_description',
                executable='pvt_to_joint',
                name='pvt_to_joint',
                output='screen',
                condition=IfCondition(conflate),
                arguments=[
                    '--endpoint', endpoint,
                    '--mode', mode,
                    '--conflate',
                    '--topic', joint_states_mapper_topic,
                    '--rate', publish_rate_hz,
                    '--save', save_csv,
                ],
            ),
        ]
    )
