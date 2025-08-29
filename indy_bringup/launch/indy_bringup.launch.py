from ament_index_python.packages import get_package_share_directory

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition  # ⬅ 추가
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

ROBOT_IPS = {
    "Indy_RB2": "192.168.0.81",
    "Indy_RC1": "192.168.0.80",
    "Moby": "192.168.214.20",
    "Nsquare": "192.168.0.83",
}

ROBOT_MODELS = {
    "Indy_RB2": "indy7",
    "Indy_RC1": "indy7",
    "Moby": "indyrp2_v2",
    "Nsquare": "indy7_v2",
}

ARGUMENTS = [
    DeclareLaunchArgument(
        "robot_name",
        default_value="Moby",
    ),
    DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Use fake hardware; when true, run joint_state_publisher_gui.",
    ),
]


def generate_launch_description():
    description_path = get_package_share_directory("indy_description")
    
    def add_robot_ip(context, *args, **kwargs):
        robot_ip = ROBOT_IPS.get(
            LaunchConfiguration("robot_name").perform(context), "0.0.0.0"
        )
        robot_description = Command([
            "xacro ", PathJoinSubstitution([FindPackageShare("indy_description"), "robots", "indy_arm.urdf.xacro"]),
            " model:=indy7_v2 arm_id:=indy"
        ])
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [description_path, "/launch/visualize_indy.launch.py"]
                ),
                launch_arguments={
                    "arm_id": "indy",
                    "model": ROBOT_MODELS.get(
                        LaunchConfiguration("robot_name").perform(context)
                    ),
                    "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                }.items(),
            ),
            Node(
                package="indy_bringup",
                executable="indy_joint_state_publisher",
                name="indy_joint_state_publisher",
                output="screen",
                parameters=[{"robot_ip": robot_ip}],
                condition=UnlessCondition(LaunchConfiguration("use_fake_hardware")), 
                remappings=[],
            ),
        ]

    # joint_state_publisher = Node(
    #     package="indy_bringup",
    #     executable="indy_joint_state_publisher",
    #     name="indy_joint_state_publisher",
    #     output="screen",
    #     parameters=[],
    #     remappings=[],
    # )

    ld = LaunchDescription(ARGUMENTS)
    # ld.add_action(joint_state_publisher)
    ld.add_action(OpaqueFunction(function=add_robot_ip))
    return ld
