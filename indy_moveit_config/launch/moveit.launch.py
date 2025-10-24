#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Full launch file that resolves `robot_name` at runtime and maps it to
ROBOT_MODELS using LaunchConfiguration.perform(context).
It also prints robot_name / model to the terminal and wires up MoveIt,
ros2_control, RViz, and the bringup include.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    LogInfo,
    Shutdown,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node


# (model, dof)
ROBOT_MODELS = {
    "Indy_RB2": ("indy7", "6"),
    "Indy_RC1": ("indy7", "6"),
    "Moby": ("indyrp2_v2", "7"),
    "Nsquare": ("indy7_v2", "6"),
}


def load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as f:
            return yaml.safe_load(f)
    except Exception:
        return None


def _setup(context, *args, **kwargs):
    # Resolve launch configs to plain strings
    robot_name = LaunchConfiguration("robot_name").perform(context)
    use_fake = LaunchConfiguration("use_fake_hardware").perform(context)
    viz = LaunchConfiguration("viz").perform(context)

    model, dof = ROBOT_MODELS.get(robot_name, ("indy7", "6"))

    # ---- robot_description (URDF) ----
    indy_xacro_file = os.path.join(get_package_share_directory("indy_description"), "robots", "indy_arm.urdf.xacro")
    robot_description_config = Command(
        [FindExecutable(name="xacro"), " ", indy_xacro_file, " model:=", model, " use_fake_hardware:=", use_fake]
    )
    robot_description = {"robot_description": robot_description_config}

    # ---- robot_description_semantic (SRDF) ----
    indy_semantic_xacro_file = os.path.join(get_package_share_directory("indy_moveit_config"), "srdf", "indy_arm.srdf.xacro")
    robot_description_semantic_config = Command(
        [FindExecutable(name="xacro"), " ", indy_semantic_xacro_file, " dof:=", dof, " hand:=", "true"]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    # ---- Kinematics / OMPL / Controllers ----
    kinematics_yaml = load_yaml("indy_moveit_config", "config/kinematics.yaml")

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/ResolveConstraintFrames "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("indy_moveit_config", "config/ompl_planning.yaml")
    if ompl_planning_yaml:
        ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml("indy_moveit_config", f"config/indy_{dof}dof_controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # ---- Nodes ----
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    rviz_config = os.path.join(get_package_share_directory("indy_moveit_config"), "rviz", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("indy_moveit_config"), "config", f"indy_{dof}dof_ros_controllers.yaml"
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[("joint_states", "indy/joint_states")],
        output={"stdout": "screen", "stderr": "screen"},
        on_exit=Shutdown(),
    )

    # Load controllers
    load_controllers = []
    for controller in ["indy_arm_controller", "joint_state_broadcaster"]:
        load_controllers.append(
            ExecuteProcess(
                cmd=[f"ros2 run controller_manager spawner {controller} --controller-manager /controller_manager"],
                shell=True,
                output="screen",
            )
        )

    bringup_path = get_package_share_directory("indy_bringup")
    joint_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                bringup_path,
                "/launch/indy_bringup.launch.py",
            ]
        ),
        launch_arguments={
            "arm_id": "indy",
            "robot_name": robot_name,
            "use_fake_hardware": use_fake,
            "viz": viz,
        }.items(),
    )

    # Informational logs
    logs = [
        LogInfo(msg=f"[indy_moveit] robot_name={robot_name}"),
        LogInfo(msg=f"[indy_moveit] model={model}"),
        LogInfo(msg=f"[indy_moveit] use_fake_hardware={use_fake}"),
        LogInfo(msg=f"[indy_moveit] viz={viz}"),
    ]

    return [
        *logs,
        robot_state_publisher,
        run_move_group_node,
        rviz_node,
        ros2_control_node,
        joint_state_publisher,
        *load_controllers,
    ]


def generate_launch_description():
    # Declare arguments with sensible defaults
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
        description="Use fake hardware",
    )
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="Moby",
        description="Select robot name to map model",
    )
    viz_arg = DeclareLaunchArgument(
        "viz",
        default_value="false",
        description="Launch RViz visualization",
    )

    return LaunchDescription(
        [
            use_fake_hardware_arg,
            robot_name_arg,
            viz_arg,
            OpaqueFunction(function=_setup),
        ]
    )
