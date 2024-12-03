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
    "Nsquare": "indyrp2",
}

ARGUMENTS = [
    DeclareLaunchArgument(
        "robot_name",
        default_value="Moby",
    )
]

def generate_launch_description():
    description_path = get_package_share_directory('indy_description')

    def add_robot_ip(context, *args, **kwargs):
        robot_ip = ROBOT_IPS.get(
            LaunchConfiguration("robot_name").perform(context), "0.0.0.0"
        )
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [description_path, "/launch/visualize_indy.launch.py"]
                ),
                launch_arguments={"arm_id": "indy",
                                  "model": ROBOT_MODELS.get(LaunchConfiguration("robot_name").perform(context))}.items(),
            ),
            Node(
                package="indy_bringup",
                executable="joint_publisher",
                name="joint_publisher",
                output="screen",
                parameters=[{"robot_ip": robot_ip}],
                remappings=[],
            )]

    # indy_description = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 [description_path, "/launch/visualize_indy.launch.py"]
    #             )
    #         )
    
    # joint_state_publisher = Node(
    #             package="indy_bringup",
    #             executable="joint_publisher",
    #             name="joint_publisher",
    #             output="screen",
    #             parameters=[],
    #             remappings=[],
    #         )
    
    ld = LaunchDescription(ARGUMENTS)
    # ld.add_action(indy_description)
    # ld.add_action(joint_state_publisher)
    ld.add_action(OpaqueFunction(function=add_robot_ip))
    return ld