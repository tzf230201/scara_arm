from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
import os
import re
import shutil
import tempfile
import uuid


def _has_jsp_gui() -> bool:
    try:
        get_package_prefix('joint_state_publisher_gui')
        return True
    except PackageNotFoundError:
        return False


def _strtobool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'y', 'on'}


def _maybe_joint_state_publisher(context, robot_description: str):
    use_joint_state_publisher = _strtobool(LaunchConfiguration('use_joint_state_publisher').perform(context))
    use_gui = _strtobool(LaunchConfiguration('use_gui').perform(context))
    joint_states_raw_topic = LaunchConfiguration('joint_states_raw_topic').perform(context)

    # If there's no GUI display available (common in headless/WSL shells), the GUI publisher will fail
    # and no JointState will be published. Fall back to the non-GUI publisher so TF stays alive.
    if use_gui and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        use_gui = False

    if not use_joint_state_publisher:
        return []

    # Always publish to joint_states_raw so the mapper owns /joint_states.
    if use_gui and _has_jsp_gui():
        return [
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen',
                parameters=[{'robot_description': robot_description}],
                remappings=[('/joint_states', joint_states_raw_topic)],
            )
        ]

    return [
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            remappings=[('/joint_states', joint_states_raw_topic)],
        )
    ]


def _make_mapper_node(context):
    joint_states_raw_topic = LaunchConfiguration('joint_states_raw_topic').perform(context)
    joint_states_topic = LaunchConfiguration('joint_states_topic').perform(context)
    return [
        Node(
            package='robot_sim',
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
        )
    ]


def _make_rviz_node(context, rviz_template_path: str, urdf_path: str):
    def _ensure_parent_dir(path: str) -> None:
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)

    def _patch_urdf_path_in_config(cfg_text: str) -> str:
        # Prefer placeholder substitution.
        if '__URDF_PATH__' in cfg_text:
            return cfg_text.replace('__URDF_PATH__', urdf_path)

        # Otherwise, patch the RobotModel "Description File" line in-place.
        # RViz configs are YAML-like; keep it simple and line-based.
        pattern = re.compile(r'^(\s*Description File:\s*)(.*)\s*$')
        lines = cfg_text.splitlines(True)
        patched_any = False
        for i, line in enumerate(lines):
            m = pattern.match(line)
            if m:
                lines[i] = f"{m.group(1)}{urdf_path}\n"
                patched_any = True
        return ''.join(lines) if patched_any else cfg_text

    user_config = LaunchConfiguration('rviz_config').perform(context).strip()

    # If the user doesn't provide a config, use a per-user persistent config file.
    # This prevents RViz settings from being lost on colcon rebuilds.
    if not user_config:
        user_config = os.path.join(os.path.expanduser('~'), '.ros', 'rviz', 'robot_sim.rviz')

        if not os.path.exists(user_config):
            _ensure_parent_dir(user_config)
            shutil.copyfile(rviz_template_path, user_config)

    # Read + patch the config in-place when possible so "Save Config" persists.
    try:
        with open(user_config, 'r', encoding='utf-8') as f:
            cfg = f.read()
        patched = _patch_urdf_path_in_config(cfg)
        if patched != cfg:
            _ensure_parent_dir(user_config)
            with open(user_config, 'w', encoding='utf-8') as f:
                f.write(patched)
        config_to_use = user_config
    except OSError:
        # Fallback: generate a temp config if the file isn't readable/writable.
        template_path = user_config if user_config else rviz_template_path
        with open(template_path, 'r', encoding='utf-8') as f:
            cfg = f.read()
        cfg = _patch_urdf_path_in_config(cfg)
        config_to_use = os.path.join(tempfile.gettempdir(), f"robot_sim_{uuid.uuid4().hex}.rviz")
        with open(config_to_use, 'w', encoding='utf-8') as f:
            f.write(cfg)

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', config_to_use],
        )
    ]


def generate_launch_description():
    # URDF comes from robot_description; RViz config is owned by robot_sim.
    desc_share = get_package_share_directory('robot_description')
    sim_share = get_package_share_directory('robot_sim')

    urdf_path = os.path.join(desc_share, 'urdf', 'robot_description.urdf')
    rviz_config_path = os.path.join(sim_share, 'rviz', 'robot_sim.rviz')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    actions = [
        # Avoid FastDDS shared-memory transport issues in WSL (open_and_lock_file failures).
        # Force UDP-only transport unless the user overrides it in their environment.
        SetEnvironmentVariable(
            name='FASTDDS_BUILTIN_TRANSPORTS',
            value=os.environ.get('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='',
            description='Path to an RViz config (.rviz).',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Use joint_state_publisher_gui when available.',
        ),
        DeclareLaunchArgument(
            'use_joint_state_publisher',
            default_value='true',
            description='Start joint_state_publisher(_gui). Disable to drive /joint_states_raw externally.',
        ),
        DeclareLaunchArgument(
            'joint_states_raw_topic',
            default_value='/joint_states_raw',
            description='Topic for raw JointState input (from GUI or external controller).',
        ),
        DeclareLaunchArgument(
            'joint_states_topic',
            default_value='/joint_states',
            description='Topic for mapped JointState output (consumed by robot_state_publisher).',
        ),
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
            remappings=[('/joint_states', LaunchConfiguration('joint_states_topic'))],
        ),
        OpaqueFunction(function=lambda context: _maybe_joint_state_publisher(context, robot_description)),
        OpaqueFunction(function=_make_mapper_node),
        OpaqueFunction(function=lambda context: _make_rviz_node(context, rviz_config_path, urdf_path)),
    ]

    return LaunchDescription(actions)
