from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os
import tempfile
import uuid


def _make_rviz_node(context, urdf_path: str, robot_description: str):
    rviz_config_template = LaunchConfiguration('rviz_config').perform(context)

    with open(rviz_config_template, 'r', encoding='utf-8') as f:
        cfg = f.read()

    cfg = cfg.replace('__URDF_PATH__', urdf_path)

    out_path = os.path.join(tempfile.gettempdir(), f"virtual_controller_{uuid.uuid4().hex}.rviz")
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(cfg)

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            arguments=['-d', out_path],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    desc_share = get_package_share_directory('robot_description')

    urdf_path = os.path.join(desc_share, 'urdf', 'robot_description.urdf')
    rviz_config_path = os.path.join(desc_share, 'rviz', 'robot_description.rviz')
    multi_motor_params_path = os.path.join(desc_share, 'config', 'multi_virtual_motor.yaml')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    cmd_topic = LaunchConfiguration('cmd_topic')
    joint_states_raw_topic = LaunchConfiguration('joint_states_raw_topic')
    joint_states_topic = LaunchConfiguration('joint_states_topic')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'rviz_config',
                default_value=rviz_config_path,
                description='Path to an RViz config (.rviz).',
            ),
            DeclareLaunchArgument(
                'cmd_topic',
                default_value='/scara_arm/tester_ui',
                description='Topic for scara_arm_msgs/TesterUICommand.',
            ),
            DeclareLaunchArgument(
                'joint_states_raw_topic',
                default_value='/joint_states_raw',
                description='Topic published by controller_virtual (raw/cumulative JointState).',
            ),
            DeclareLaunchArgument(
                'joint_states_topic',
                default_value='/joint_states',
                description='Topic for mapped JointState output.',
            ),
            DeclareLaunchArgument(
                'with_ui',
                default_value='true',
                description='Also launch robot_ui tester_ui.',
            ),
            DeclareLaunchArgument(
                'with_virtual_motors',
                default_value='false',
                description=(
                    'Also launch robot_description/multi_virtual_motor (requires that executable + config exist in robot_description).'
                ),
            ),

            # Make tester_ui publish to the same cmd_topic.
            SetEnvironmentVariable(name='TESTER_UI_TOPIC', value=cmd_topic),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='world_to_base_tf',
                output='screen',
                arguments=['0', '0', '0', '0', '0', '0', 'world', 'Fixed_Vertical_Rail'],
            ),

            Node(
                package='robot_ui',
                executable='tester_ui',
                name='tester_ui',
                output='screen',
                condition=IfCondition(LaunchConfiguration('with_ui')),
            ),

            Node(
                package='robot_description',
                executable='multi_virtual_motor',
                name='multi_virtual_motor',
                output='screen',
                condition=IfCondition(LaunchConfiguration('with_virtual_motors')),
                parameters=[multi_motor_params_path],
            ),

            Node(
                package='robot_controller_virtual',
                executable='controller_virtual',
                name='controller_virtual',
                output='screen',
                parameters=[
                    {'cmd_topic': cmd_topic},
                    {'joint_states_raw_topic': joint_states_raw_topic},
                    {'publish_rate_hz': 50.0},
                    {'joint1_input_unit': 'mm'},
                    {'ui_joint_angle_space': 'motor_simple'},
                ],
            ),

            Node(
                package='robot_controller_virtual',
                executable='joint_state_mapper',
                name='joint_state_mapper',
                output='screen',
                parameters=[
                    {'input_topic': joint_states_raw_topic},
                    {'output_topic': joint_states_topic},
                    {'joint1_name': 'joint_1'},
                    {'joint2_name': 'joint_2'},
                    {'joint3_name': 'joint_3'},
                    {'joint4_name': 'joint_4'},
                ],
            ),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description}],
                remappings=[('/joint_states', joint_states_topic)],
            ),

            OpaqueFunction(function=lambda context: _make_rviz_node(context, urdf_path, robot_description)),
        ]
    )
