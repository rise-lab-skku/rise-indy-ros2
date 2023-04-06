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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
import os


def generate_launch_description():
    description_package = get_package_share_directory("indy_description")

    # Declare the launch arguments
    arm_id_arg = DeclareLaunchArgument(
        "arm_id",
        default_value="indy",
        description='Arm instance name used as joint name prefix for multi-robot setup. \
            Example: "indy_L" and "indy_R" for two indy7 robots.',
    )
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="indy7",
        description='Robot model should be the folder name under "robots" and "meshes" folder. \
            Example: "indy7" or "indyrp2_v2"',
    )

    # Path
    xacro_path = os.path.join(description_package, "robots", "indy_arm.urdf.xacro")
    rviz_config = os.path.join(description_package, "rviz", "visualize_indy.rviz")

    # Nodes
    arm_id = LaunchConfiguration("arm_id")
    model = LaunchConfiguration("model")
    xacro_options = [" arm_id:=", arm_id, " model:=", model]
    robot_description = Command([FindExecutable(name="xacro"), " ", xacro_path] + xacro_options)

    return LaunchDescription(
        [
            arm_id_arg,
            model_arg,
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
            ),
        ]
    )
