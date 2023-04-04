from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
import os


def generate_launch_description():
    description_package = get_package_share_directory('indy_description')

    # Declare the launch arguments
    arm_id_arg = DeclareLaunchArgument(
        'arm_id',
        default_value="indy",
        description='Arm instance name used as joint name prefix for multi-robot setup. Example: "indy_L" and "indy_R" for two indy7 robots.'
    )
    model_arg = DeclareLaunchArgument(
        'model',
        default_value="indy7",
        description='Robot model should be the folder name under "robots" and "meshes" folder. Example: "indy7" or "indyrp2_v2"'
    )
    # prefix_arg = DeclareLaunchArgument(
    #     'prefix',
    #     default_value='""',
    #     description='Prefix for joint names in multi-robot setup; If the prefix is changed, controller configurations also be updated.'
    # )

    # Path
    xacro_path = os.path.join(description_package, 'robots', 'indy_arm.urdf.xacro')
    rviz_config = os.path.join(description_package, 'rviz', 'visualize_indy.rviz')

    # Nodes
    arm_id = LaunchConfiguration('arm_id')
    model = LaunchConfiguration('model')
    xacro_options = [" arm_id:=", arm_id, " model:=", model]
    robot_description = Command([FindExecutable(name='xacro'), ' ', xacro_path] + xacro_options)

    return LaunchDescription([
        arm_id_arg,
        model_arg,
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
