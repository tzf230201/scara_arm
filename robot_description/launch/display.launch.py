from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')

    urdf_path = os.path.join(pkg_share, 'urdf', 'robot_description.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'robot_description.rviz')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    nodes = [
        # Ensure RViz always has a stable fixed frame.
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
    ]

    # Prefer the GUI if available, otherwise fall back to the non-GUI publisher.
    try:
        get_package_prefix('joint_state_publisher_gui')
        nodes.append(
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen',
            )
        )
    except PackageNotFoundError:
        nodes.append(
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
            )
        )

    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        )
    )

    return LaunchDescription(nodes)
