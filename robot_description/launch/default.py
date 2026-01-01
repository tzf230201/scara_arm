from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
import os
import tempfile
import uuid


def _make_rviz_node(context, urdf_path: str):
    rviz_config_template = LaunchConfiguration('rviz_config').perform(context)

    with open(rviz_config_template, 'r', encoding='utf-8') as f:
        cfg = f.read()

    cfg = cfg.replace('__URDF_PATH__', urdf_path)

    out_path = os.path.join(tempfile.gettempdir(), f"robot_description_{uuid.uuid4().hex}.rviz")
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(cfg)

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', out_path],
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')

    urdf_path = os.path.join(pkg_share, 'urdf', 'robot_description.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'robot_description.rviz')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    nodes = [
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_path,
            description='Path to an RViz config (.rviz).',
        ),
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
        # If there's no GUI display available (common in headless/WSL shells),
        # the GUI publisher will fail. Fall back to the non-GUI publisher.
        if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
            nodes.append(
                Node(
                    package='joint_state_publisher_gui',
                    executable='joint_state_publisher_gui',
                    name='joint_state_publisher_gui',
                    output='screen',
                    parameters=[{'robot_description': robot_description}],
                )
            )
        else:
            nodes.append(
                Node(
                    package='joint_state_publisher',
                    executable='joint_state_publisher',
                    name='joint_state_publisher',
                    output='screen',
                    parameters=[{'robot_description': robot_description}],
                )
            )
    except PackageNotFoundError:
        nodes.append(
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description}],
            )
        )

    nodes.append(OpaqueFunction(function=lambda context: _make_rviz_node(context, urdf_path)))

    return LaunchDescription(nodes)
