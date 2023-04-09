# Copyright 2023 RISE LAB, Neuromeka Co.,Ltd.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the RISE LAB, Neuromeka Co.,Ltd. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

# To check the launch arguments, run:
#   ros2 launch -s indy_sim_gazebo sim.launch.py


def generate_launch_description():
    # Required packages
    pkg_indy_sim_gazebo = get_package_share_directory("indy_sim_gazebo")
    pkg_gz_sim = get_package_share_directory("ros_gz_sim")

    # Declare the launch arguments
    arm_id_arg = DeclareLaunchArgument(
        "arm_id",
        default_value="indy",
        description='Arm instance name used as joint name prefix for multi-robot setup. Example: "indy_L" and "indy_R" for two indy7 robots.',
    )
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="indy7",
        description='Robot model should be the folder name under "robots" and "meshes" folder. Example: "indy7" or "indyrp2_v2"',
    )
    arm_id = LaunchConfiguration("arm_id")
    model = LaunchConfiguration("model")

    ###### Node:robot_state_publisher (Load robot model)
    xacro_path = os.path.join(pkg_indy_sim_gazebo, "models", "indy.xacro")
    xacro_options = [" arm_id:=", arm_id, " model:=", model]
    robot_description = Command([FindExecutable(name="xacro"), " ", xacro_path] + xacro_options)

    ###### Gazebo
    world = os.path.join(pkg_indy_sim_gazebo, "worlds", "empty.sdf")
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "/robot_description", "-name", "indy"],
        output="screen",
        # See `$ ros2 run ros_gz_sim create --helpshort` for more options
    )
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # Clock (ROS2 <- IGN)
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # Joint states (ROS2 <- IGN)
            "/world/empty/model/indy/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        remappings=[
            ("/world/empty/model/indy/joint_state", "joint_states"),
        ],
        output="screen",
    )

    ###### ROS2 control
    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare("ros2_control_demo_example_1"),
    #         "config",
    #         "rrbot_controllers.yaml",
    #     ]
    # )
    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[{"robot_description": robot_description}, robot_controllers],
    #     output="both",
    # )
    # load_joint_state_broadcaster = ExecuteProcess(
    #     cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"],
    #     output="screen",
    # )
    # load_joint_trajectory_controller = ExecuteProcess(
    #     cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_position_trajectory_controller"],
    #     output="screen",
    # )

    ###### Node:rviz2
    rviz_config = os.path.join(pkg_indy_sim_gazebo, "rviz", "sim.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="log",
    )

    return LaunchDescription(
        [
            # SetEnvironmentVariable(
            #     "RCUTILS_CONSOLE_OUTPUT_FORMAT",
            #     "[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})",
            # ),
            ###### Node:robot_state_publisher (Load robot model)
            arm_id_arg,
            model_arg,
            Node(  # TF broadcaster
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": True, "robot_description": robot_description}],
            ),
            ###### Gazebo
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_gz_sim, "launch", "gz_sim.launch.py")),
                launch_arguments={"gz_args": f"-r {world}"}.items(),
                # See `gz sim --help` for more options
            ),
            TimerAction(period=2.0, actions=[gz_spawn_entity]),
            RegisterEventHandler(event_handler=OnProcessStart(target_action=gz_spawn_entity, on_start=[gz_bridge])),
            ###### ROS2 control
            # control_node,
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=gz_spawn_entity,
            #         on_exit=[load_joint_state_broadcaster],
            #     )
            # ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=load_joint_state_broadcaster,
            #         on_exit=[load_joint_trajectory_controller],
            #     )
            # ),
            ###### Node:rviz2
            RegisterEventHandler(event_handler=OnProcessStart(target_action=gz_bridge, on_start=[rviz])),
        ]
    )
