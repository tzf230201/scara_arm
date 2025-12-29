from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Reuse the URDF + RViz config from robot_description.
    desc_share = get_package_share_directory('robot_description')

    urdf_path = os.path.join(desc_share, 'urdf', 'robot_description.urdf')
    rviz_config_path = os.path.join(desc_share, 'rviz', 'robot_description.rviz')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Provide a stable fixed frame for RViz.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_tf',
            output='screen',
            # x y z yaw pitch roll frame_id child_frame_id
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'Fixed_Vertical_Rail'],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        # No GUI: joint_state_publisher only.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),
    ])
