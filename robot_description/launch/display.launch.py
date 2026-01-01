from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    desc_share = get_package_share_directory("robot_description")

    urdf_path = os.path.join(desc_share, "urdf", "robot_description.urdf")
    rviz_config_path = os.path.join(desc_share, "rviz", "robot_description.rviz")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    actions = [
        DeclareLaunchArgument(
            "rviz_config",
            default_value=rviz_config_path,
            description="Path to an RViz config (.rviz).",
        ),
        DeclareLaunchArgument(
            "joint_states_raw_topic",
            default_value="/joint_states_raw",
            description="Topic for raw JointState (from joint_state_publisher_gui).",
        ),
        DeclareLaunchArgument(
            "joint_states_topic",
            default_value="/joint_states",
            description="Topic for mapped JointState output.",
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_base_tf",
            output="screen",
            arguments=["0", "0", "0", "0", "0", "0", "world", "Fixed_Vertical_Rail"],
        ),

        # GUI publisher: selalu dipakai
        # Node(
        #     package="joint_state_publisher",
        #     executable="joint_state_publisher",
        #     name="joint_state_publisher",
        #     output="screen",
        #     parameters=[{"robot_description": robot_description}],
        #     remappings=[("/joint_states", LaunchConfiguration("joint_states_raw_topic"))],
        # ),

        # mapper kamu
        Node(
            package="robot_description",
            executable="joint_state_mapper",
            name="joint_state_mapper",
            output="screen",
            parameters=[
                {"input_topic": LaunchConfiguration("joint_states_raw_topic")},
                {"output_topic": LaunchConfiguration("joint_states_topic")},
                {"joint1_name": "joint_1"},
                {"joint2_name": "joint_2"},
                {"joint3_name": "joint_3"},
                {"joint4_name": "joint_4"},
            ],
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            remappings=[("/joint_states", LaunchConfiguration("joint_states_topic"))],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ]

    return LaunchDescription(actions)
