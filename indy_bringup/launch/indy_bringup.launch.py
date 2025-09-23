# 필요한 모듈들을 가져옵니다.
from ament_index_python.packages import get_package_share_directory

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    LogInfo,
    TimerAction,  # ⬅ TimerAction을 추가합니다.
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# 로봇 이름에 따른 IP와 모델명 딕셔너리
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

# 런치 인자 정의
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
    DeclareLaunchArgument(
        "viz",
        default_value="true",
        description="Use RViz visualization.",
    ),
    DeclareLaunchArgument(
        "use_hand",
        default_value="false",
        description="Use hand",
    ),
]


def generate_launch_description():
    description_path = get_package_share_directory("indy_description")
    amm_controller_path = get_package_share_directory("amm_controller")

    def add_robot_ip(context, *args, **kwargs):
        # 런치 인자 값 가져오기
        robot_name = LaunchConfiguration("robot_name").perform(context)
        robot_ip = ROBOT_IPS.get(robot_name, "0.0.0.0")
        model = ROBOT_MODELS.get(robot_name)

        # 실행할 액션들을 담을 리스트 생성
        actions_to_execute = []

        # === 단계 1: 즉시 실행 (T=0초) ===
        # 로그 메시지와 handeye_calibration 런치 파일 포함
        actions_to_execute.extend(
            [
                LogInfo(msg=f"[indy_bringup] robot_name={robot_name}"),
                LogInfo(msg=f"[indy_bringup] robot_ip={robot_ip}"),
                LogInfo(msg=f"[indy_bringup] model={model}"),
                LogInfo(msg="[indy_bringup] Launching handeye_calibration..."),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([amm_controller_path, "/launch/handeye_calibration.launch.py"]),
                ),
            ]
        )

        # TimerAction을 사용하여 indy_joint_state_publisher 노드를 6초 지연 실행
        actions_to_execute.append(
            TimerAction(
                period=1.0,
                actions=[
                    LogInfo(msg="[indy_bringup] Waited another 3 seconds. Launching indy_joint_state_publisher..."),
                    Node(
                        package="indy_bringup",
                        executable="indy_joint_state_publisher",
                        name="indy_joint_state_publisher",
                        output="screen",
                        parameters=[{"robot_ip": robot_ip}],
                        condition=UnlessCondition(LaunchConfiguration("use_fake_hardware")),
                        remappings=[],
                    ),
                ],
            )
        )

        actions_to_execute.append(
            TimerAction(
                period=1.0,
                actions=[
                    LogInfo(msg="[indy_bringup] Waited 3 seconds. Launching visualization..."),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([description_path, "/launch/visualize_indy.launch.py"]),
                        launch_arguments={
                            "arm_id": "indy",
                            "model": model,
                            "robot_ip": robot_ip,
                            "use_hand": LaunchConfiguration("use_hand"),
                            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                            "viz": LaunchConfiguration("viz"),
                        }.items(),
                    ),
                ],
            )
        )

        return actions_to_execute

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=add_robot_ip))
    return ld
