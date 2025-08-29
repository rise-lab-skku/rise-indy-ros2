#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time
from rclpy.qos import QoSProfile, DurabilityPolicy

from vision_msgs.msg import Detection3D, Detection3DArray
from geometry_msgs.msg import Pose, PoseArray, PoseStamped

# from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

# from tf2_ros import TransformBroadcaster

# Debug
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


def marker_color(r, g, b, a):
    c = ColorRGBA()
    c.r = r
    c.g = g
    c.b = b
    c.a = a
    return c


RED = marker_color(1.0, 0.0, 0.0, 1.0)
GREEN = marker_color(0.0, 1.0, 0.0, 1.0)
BLUE = marker_color(0.0, 0.0, 1.0, 1.0)
YELLOW = marker_color(1.0, 1.0, 0.0, 1.0)
IDLE = 5

# Indy
import numpy as np
from scipy.spatial.transform import Rotation
import time
from neuromeka import IndyDCP3

# Robot IP
Indy_RB2 = "192.168.0.81"
Indy_RC1 = "192.168.0.80"
Moby = "192.168.214.20"
Nsquare = "192.168.0.83"

class IndyCommander(Node):
    def __init__(self):
        super().__init__("indy_commander")

        # ROS subscriber
        self.cgn_detection_subscriber = self.create_subscription(Pose, "/cgn_grasp_point", self.cgn_detection_cb, 2)
        self.pvn_detection_subscriber = self.create_subscription(
            # PoseArray, "/pvn_grasp_point", self.pvn_detection_cb, 2
            PoseArray,
            "/grasp_poses",
            self.pvn_detection_cb,
            2,
        )

        # tf publisher
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # self.tf_broadcaster = TransformBroadcaster(self)
        self.grasp_tf_publisher = self.create_publisher(PoseStamped, "/indy_grasp_pose", qos_profile=qos)
        self.grasp_cands_publisher = self.create_publisher(PoseArray, "/indy_grasp_candidates", qos_profile=qos)
        self._grasp_debug_publisher = self.create_publisher(MarkerArray, "/indy_grasp_debug", 10)
        self._debug_marker_array = MarkerArray()
        self._debug_marker_max_id = 0

        # read ros parameters
        self.declare_parameter("robot_ip", Moby)
        self.robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value

        # connect to indy
        self.indy = IndyDCP3(self.robot_ip)

        # IDLE pose (파지 박스 위치에 따라 달라질 수 있음)
        # self.idle_joint_pose = [-16.22726, 10.679438, -10.180241, 49.00724, -6.1019807, 133.21788, -30.117714]
        # self.idle_pose_z = 801.0
        self.idle_joint_pose = [-10.901025, 8.16611, -14.447203, 80.84503, -4.1680064, 104.15691, -26.003605]
        self.idle_pose_z = 696.7
        self.indy.movej(self.idle_joint_pose, vel_ratio=3, acc_ratio=100)

        # 파지할 때 바닥에 충돌 안하는 최소 위치
        self.goal_min_z = 490.0

        # hand_palm to indy_flange_nrmkapi
        self.flange2palm = np.array(
            [
                [-0.000, -0.866, -0.500, 0.088],
                [-0.000, -0.500, 0.866, -0.198],
                [-1.000, 0.000, 0.000, 0.001],
                [0.000, 0.000, 0.000, 1.000]
            ]
        )
        self.palm2flange = np.linalg.inv(self.flange2palm)

        # camera_color_optical_frame to camera_link (Fixed TF)
        self.cam2camlink = np.array(
            [
                [0.0, 0.0, 1.0, -0.002],
                [-1.0, 0.0, 0.0, -0.033],
                [0.0, -1.0, 0.0, -0.001],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        # camera_link to indy_flange_nrmkapi (Fixed TF)
        self.cam_orien = Rotation.from_quat(
            [
                0.7924110610356948,
                0.0014661680130881364,
                0.6099747332796601,
                -0.0036586144985633474,
            ]
        ).as_euler("xyz", degrees=True)

        self.camlink2ee = self.euler2tfmatrix(
            0.09696821729879207,
            -0.0010907832847846174,
            0.06806743694805707,
            self.cam_orien[0],
            self.cam_orien[1],
            self.cam_orien[2],
        )

        # 250829 camera_optical_frame to indy_flange by MoveitCalibration
        self.camlink2ee = self.euler2tfmatrix(
            0.105418,
            -0.038445,
            0.0553039,
            0.024639,
            0.518373,
            -1.57238,
        )

        # camera_color_optical_frame to indy_flange_nrmkapi
        self.cam2ee = np.dot(self.camlink2ee, self.cam2camlink)

        # variables
        self.grasp_per_object = 100

    def euler2tfmatrix(self, tx, ty, tz, roll, pitch, yaw):
        tf = np.eye(4)
        tf[:3, 3] = np.array([tx, ty, tz])
        tf[:3, :3] = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=True).as_matrix()
        return tf

    def pose2tfmatrix(self, pose: Pose):
        tf = np.eye(4)
        tf[:3, 3] = np.array([pose.position.x, pose.position.y, pose.position.z])
        tf[:3, :3] = Rotation.from_quat(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        ).as_matrix()
        return tf

    def indy2tfmatrix(self):
        indy_pose = np.array(self.indy.get_control_data()["p"])
        indy_pose[:3] *= 0.001

        return self.euler2tfmatrix(
            indy_pose[0],
            indy_pose[1],
            indy_pose[2],
            indy_pose[3],
            indy_pose[4],
            indy_pose[5],
        )

    def tfmatrix2indy(self, tf: np.ndarray):
        # m, rad -> mm, deg
        indy_pose = np.zeros(6)
        indy_pose[:3] = tf[:3, 3] * 1000
        indy_pose[3:] = Rotation.from_matrix(tf[:3, :3]).as_euler("xyz", degrees=True)
        return indy_pose

    def alignment_score(self, pose: Pose):
        # check if pose is aligned with global -z axis
        # Extract quaternion
        qx, qy, qz, qw = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        # Compute rotation matrix from quaternion
        rot_mat = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()

        # Z-axis in the local frame
        return rot_mat[:, 1][2]

    def make_tf(self, pose: Pose, frame_id: str, child_frame_id: str):
        # tf = TransformStamped()
        # tf.header.stamp = self.get_clock().now().to_msg()
        # tf.header.frame_id = frame_id
        # tf.child_frame_id = child_frame_id
        # tf.transform.translation.x = pose.position.x
        # tf.transform.translation.y = pose.position.y
        # tf.transform.translation.z = pose.position.z
        # tf.transform.rotation = pose.orientation
        # self.tf_broadcaster.sendTransform(tf)
        # self.grasp_tf_publisher.publish(tf)
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = pose
        self.grasp_tf_publisher.publish(msg)

    def move_from_detection_pvn(self, detection):

        # PVN용 쫙 벌리기 (FULL OPEN 보다는 조금 작은)
        self.hand_interface.move_in_deg((0.0, 27.0, 15.0), (0.0, 27.0, 15.0), (0.0, 27.0, 15.0), 1.0)

        # "camera_color_optical_frame" 에서 인식된 파지 자세를 받아 로봇을 이동
        # camera_color_optical_frame 기준 ee의 위치
        detection_ee = np.dot(detection, self.flange2palm)
        # z축이 파지 접근 방향 (CGN 기준)
        # goal_pose_ee:
        goal_pose_ee = np.dot(self.cam2ee, detection_ee)


        # change frame indy_flange_nrmkapi to indy_link0
        # m, rad
        ee2base = self.indy2tfmatrix()
        goal_pose_base = np.dot(ee2base, goal_pose_ee)

        # goal_pose_base_palm = np.dot(self.flange2palm, goal_pose_base)
        print("goal pose:", goal_pose_base, flush=True)

        # TODO: goal_pose_base 위치에 dummy hand rviz 시각화
        goal_pose_indy = self.tfmatrix2indy(goal_pose_base)

        # test movement
        goal_z_pose = goal_pose_indy[2]
        goal_pose_indy[2] = self.idle_pose_z

        user_input = input("Type 'y' to move robot or type any key to pass: ").strip().lower()
        if user_input == "y":
            print("Move Indy")
            # move to target x y position
            self.indy.movel(goal_pose_indy, vel_ratio=10, acc_ratio=100)
            # go down
            goal_pose_indy[2] = goal_z_pose
            time.sleep(1.0)
            self.indy.movel(goal_pose_indy, vel_ratio=3, acc_ratio=100)

            # 도착까지 대기 (대기 없으면 바로 다음 움직임으로 넘어감)
            indy_state = self.indy.get_robot_data()["op_state"]
            while True:
                time.sleep(0.1)
                indy_state = self.indy.get_robot_data()["op_state"]
                if indy_state == IDLE:
                    break

            # TODO 파지
            # self.hand_interface.move_pose(HandPose.PINCH_CLOSE)
            self.hand_interface.move_pose(HandPose.BASIC_CLOSE)

            # go up
            goal_pose_indy[2] = self.idle_pose_z
            time.sleep(1.0)
            self.indy.movel(goal_pose_indy, vel_ratio=3, acc_ratio=100)

            # 내려놓기 중간지점
            self.indy.movej(
                [-72.07381, 37.192715, -24.84045, 46.57486, 14.906924, 98.6253, -89.97912],
                vel_ratio=5,
                acc_ratio=100,
            )

            # 내려놓기 위치
            self.indy.movej(
                [-71.45199, 41.275635, -16.942825, 83.49378, 13.251035, 57.23877, -91.570175],
                vel_ratio=3,
                acc_ratio=100,
            )
            time.sleep(0.1)

            # 도착까지 대기 (대기 없으면 바로 다음 움직임으로 넘어감)
            while True:
                time.sleep(0.1)
                indy_state = self.indy.get_robot_data()["op_state"]
                if indy_state == IDLE:
                    break

            # 그리퍼 파지 해제 (물건 놓기)
            # TODO 옮기고, 파지 해제
            self.hand_interface.move_pose(HandPose.BASIC_OPEN)
            # self.hand_interface.move_pose(HandPose.FULL_OPEN)

            # 대기위치 중간지점
            self.indy.movej(
                [-72.07381, 37.192715, -24.84045, 46.57486, 14.906924, 98.6253, -89.97912],
                vel_ratio=5,
                acc_ratio=100,
            )

            # 대기위치
            self.indy.movej(
                [-16.22726, 10.679438, -10.180241, 49.00724, -6.1019807, 133.21788, -30.117714],
                vel_ratio=5,
                acc_ratio=100,
            )
        else:
            pass

    def move_from_detection_cgn(self, detection):

        """
        # 시저 파지로 franka 파지 폭 맞추기
        오픈
        hand_interface.move_in_deg((0.0, -60.0, -75.0), (-17.0, -30.0, 0.0), (17.0, -30.0, 0.0), 0.7)
        클로즈
        hand_interface.move_in_deg((0.0, -60.0, -75.0), (20.0, -30.0, 0.0), (-20.0, -30.0, 0.0), 0.5)
        """
        def franka_open():
            self.hand_interface.move_in_deg((0.0, -60.0, -75.0), (-14.0, -30.0, 0.0), (14.0, -30.0, 0.0), 0.7)
        def franka_close():
            self.hand_interface.move_in_deg((0.0, -60.0, -75.0), (20.0, -30.0, 0.0), (-20.0, -30.0, 0.0), 0.5)

        # "camera_color_optical_frame" 에서 인식된 파지 자세를 받아 로봇을 이동

        # detection x축 정렬
        # TODO: CGN은 z축이 접근방향이라 변환 필요
        # detection_aligned = np.dot(detection, self.euler2tfmatrix(0.0, 0.0, 0.0, 0.0, 0.0, -90.0))

        # fix orientation for demo
        detection_euler = Rotation.from_matrix(detection[:3, :3]).as_euler("xyz", degrees=True)
        # TODO: 오일러 각도 주의
        detection_euler[0] = 0.0
        detection_euler[1] = 0.0

        if detection_euler[2] > 0.0 and detection_euler[2] < 180.0:
            detection_euler[2] -= 180.0

        # add roll 30 degree for amm hand
        detection[:3, :3] = Rotation.from_euler("xyz", detection_euler, degrees=True).as_matrix()
        # detection = np.dot(detection, self.euler2tfmatrix(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        # camera_color_optical_frame 기준 ee의 위치
        # z축이 파지 접근 방향 (CGN 기준)
        # goal_pose_ee:
        goal_pose_ee = np.dot(self.cam2ee, detection)


        # print(
        #     "goal pose euler: ",
        #     Rotation.from_matrix(goal_pose_ee[:3, :3]).as_euler("xyz", degrees=True),
        #     flush=True,
        # )

        # change frame indy_flange_nrmkapi to indy_link0
        # m, rad
        ee2base = self.indy2tfmatrix()
        goal_pose_base = np.dot(ee2base, goal_pose_ee)

        # goal_pose_base_palm = np.dot(self.flange2palm, goal_pose_base)
        print("goal pose:", goal_pose_base, flush=True)

        # TODO: goal_pose_base 위치에 dummy hand rviz 시각화
        # goal_pose_indy = self.tfmatrix2indy(goal_pose_base)
        # goal_pose_indy = self.tfmatrix2indy(goal_pose_base)
        goal_pose_indy = self.tfmatrix2indy(goal_pose_base)

        # offset hardcoded
        goal_pose_indy[0] += 8
        goal_pose_indy[2] -= 10

        # test movement
        goal_z_pose = goal_pose_indy[2]
        goal_pose_indy[2] = self.idle_pose_z


        user_input = input("Type 'y' to move robot or type any key to pass: ").strip().lower()
        if user_input == "y":
            print("Move Indy")
            # CGN용 Franka Gripper 폭 맞추기
            franka_open()
            time.sleep(2.0)
            # move to target x y position
            self.indy.movel(goal_pose_indy, vel_ratio=10, acc_ratio=100)
            # go down
            goal_pose_indy[2] = goal_z_pose + 250.0
            if goal_pose_indy[2] < self.goal_min_z:
                goal_pose_indy[2] = self.goal_min_z
            time.sleep(1.0)
            self.indy.movel(goal_pose_indy, vel_ratio=3, acc_ratio=100)

            # 도착까지 대기 (대기 없으면 바로 다음 움직임으로 넘어감)
            indy_state = self.indy.get_robot_data()["op_state"]
            while True:
                time.sleep(0.1)
                indy_state = self.indy.get_robot_data()["op_state"]
                if indy_state == IDLE:
                    break

            # TODO 파지
            franka_close()

            # go up
            goal_pose_indy[2] = self.idle_pose_z
            time.sleep(1.0)
            self.indy.movel(goal_pose_indy, vel_ratio=3, acc_ratio=100)

            # # 내려놓기 중간지점
            # self.indy.movej(
            #     [-72.07381, 37.192715, -24.84045, 46.57486, 14.906924, 98.6253, -89.97912],
            #     vel_ratio=5,
            #     acc_ratio=100,
            # )

            # # 내려놓기 위치
            # self.indy.movej(
            #     [-71.45199, 41.275635, -16.942825, 83.49378, 13.251035, 57.23877, -91.570175],
            #     vel_ratio=3,
            #     acc_ratio=100,
            # )

            # 내려놓기 위치
            self.indy.movej(
                [-70.89419, 36.32684, -23.825941, 55.205845, 13.878071, 90.87157, -90.21712],
                vel_ratio=3,
                acc_ratio=100,
            )

            time.sleep(0.5)

            # 도착까지 대기 (대기 없으면 바로 다음 움직임으로 넘어감)
            while True:
                time.sleep(0.1)
                indy_state = self.indy.get_robot_data()["op_state"]
                if indy_state == IDLE:
                    break

            # 그리퍼 파지 해제 (물건 놓기)
            # TODO 옮기고, 파지 해제
            franka_open()
            time.sleep(1.0)
            # 대기위치 중간지점
            self.indy.movej(
                [-72.07381, 37.192715, -24.84045, 46.57486, 14.906924, 98.6253, -89.97912],
                vel_ratio=5,
                acc_ratio=100,
            )

            # 대기위치
            self.indy.movej(
                self.idle_joint_pose,
                vel_ratio=5,
                acc_ratio=100,
            )

            self.hand_interface.move_pose(HandPose.BASIC_CLOSE)
        else:
            pass

    def cgn_detection_cb(self, msg: Pose):
        print("ContactGraspNet detection received")
        # change Pose to Transformation matrix
        goal_pose_camera = self.pose2tfmatrix(msg)

        # move with detection result
        # Z 방향이 파지하러 접근하는 방향
        # X 방향이 finger 1 방향
        #
        # TODO: goal_pose_camera를 world 기준으로 변경해서 넣어야 함.
        self.move_from_detection_cgn(goal_pose_camera)

    def _pub_debug_markers(self):
        num_current = len(self._debug_marker_array.markers)
        if num_current > self._debug_marker_max_id:
            self._debug_marker_max_id = num_current
        else:
            for i in range(num_current, self._debug_marker_max_id):
                # Remove old markers
                rm = Marker()
                rm.action = Marker.DELETE
                rm.header.frame_id = "camera_color_optical_frame"
                self._debug_marker_array.markers.append(rm)
        for i in range(self._debug_marker_max_id):
            self._debug_marker_array.markers[i].header.stamp = self.get_clock().now().to_msg()
            self._debug_marker_array.markers[i].id = i
        self._grasp_debug_publisher.publish(self._debug_marker_array)
        self._debug_marker_max_id = num_current
        self._debug_marker_array.markers = []

    def pvn_detection_cb(self, pose_array: PoseArray):
        print("PVN3D detection received")
        # check object number
        poses_num = int(len(pose_array.poses))
        object_num = poses_num // self.grasp_per_object

        # find best aligned pose for each object
        # best_poses = np.empty(int(object_num))

        # pose_array: camera_color_optical_frame 기준. grasp poses

        # 파지 자세 필터링
        deg_lim = 30  # 각도 제한
        filtered_poses = []

        for pose in pose_array.poses:
            # pose를 변환 행렬로 변환
            pose_mat = self.pose2tfmatrix(pose)

            # x축 벡터의 두번째 원소 추출 (x축은 첫번째 열)
            x_axis_first = pose_mat[1, 0]

            _debug_color = RED
            _debug_color.a = 0.5
            # 각도 제한 내에 있는지 확인
            if x_axis_first > np.cos(np.deg2rad(deg_lim)):
                filtered_poses.append(pose)
                _debug_color = GREEN

            _debug_marker = Marker()
            _debug_marker.header.frame_id = "camera_color_optical_frame"
            _debug_marker.type = Marker.SPHERE
            _debug_marker.action = Marker.ADD
            _debug_marker.pose = pose
            _debug_marker.scale.x = 0.005
            _debug_marker.scale.y = 0.005
            _debug_marker.scale.z = 0.005
            _debug_marker.color = _debug_color
            self._debug_marker_array.markers.append(_debug_marker)

        if len(filtered_poses) == 0:
            self.get_logger().warn("No valid grasp poses found")
            filtered_poses = pose_array.poses

        # DEBUG
        # self.get_logger().warn("pub debug markers")
        # self._pub_debug_markers()

        # find min z pose
        # min_z_pose = min(pose_array.poses, key=lambda x: x.position.z)
        min_z_pose = min(filtered_poses, key=lambda x: x.position.z)

        # best_poses = {}
        # alignment_score = 0.0

        # _obj_order = 0
        # self.get_logger().warn(f"{poses_num = }")
        # for idx in range(0, poses_num, self.grasp_per_object):
        #     if (idx + self.grasp_per_object) % self.grasp_per_object != 0:
        #         break
        #     for pose in pose_array.poses[idx : idx + self.grasp_per_object]:
        #         score = self.alignment_score(pose)
        #         if score > alignment_score:
        #             alignment_score = score
        #             # best_poses[idx // self.grasp_per_object] = pose
        #             best_poses[_obj_order] = pose
        #             self.get_logger().warn(f"{best_poses = }")
        #     alignment_score = 0.0
        #     _obj_order += 1

        # # find minimum z grasp pose
        # # best_poses를 PoseArray로 변환하여 발행
        # grasp_cands_msg = PoseArray()
        # grasp_cands_msg.header.frame_id = "camera_color_optical_frame"
        # grasp_cands_msg.header.stamp = self.get_clock().now().to_msg()
        # grasp_cands_msg.poses = list(best_poses.values())
        # self.grasp_cands_publisher.publish(grasp_cands_msg)

        # min_z_pose = None
        # min_z = np.inf
        # for pose in best_poses.values():
        #     print("z pose: ", pose.position.z, flush=True)
        #     if pose.position.z < min_z:
        #         min_z = pose.position.z
        #         min_z_pose = pose

        # make tf for visualization
        # for idx, pose in enumerate(best_poses):
        parent_frame = "camera_color_optical_frame"
        #     child_frame = f"object_{idx}"
        #     if pose == max_z_pose:
        child_frame = "grasp_result"
        self.make_tf(min_z_pose, parent_frame, child_frame)  # TODO: stastic tf?

        # change frame camera_color_optical_frame to indy_flange_nrmkapi
        # goal_pose_camera_pvn = self.pose2tfmatrix(min_z_pose)
        goal_pose_camera = self.pose2tfmatrix(min_z_pose)
        print("goal pose camera: ", goal_pose_camera, flush=True)
        # rotate 90 degree to match with CGN

        # Y 방향이 파지하러 접근하는 방향
        # X 방향이 finger 1 방향
        # ----- 접근 방향 달라서 변환 -----
        # PVN결과는 palm 기준으로 핸드 Y축 내려가는 방향인데, eef가 z축 방향이 내려가는 방향이라서 돌림
        # goal_pose_camera = np.dot(goal_pose_camera_pvn, self.euler2tfmatrix(0.0, 0.0, 0.0, -90.0, 0.0, 0.0))

        # 4x4 transformation matrix를 ROS2 Pose 메시지로 변환
        goal_pose_msg = Pose()

        # position 설정
        goal_pose_msg.position.x = goal_pose_camera[0, 3]
        goal_pose_msg.position.y = goal_pose_camera[1, 3]
        goal_pose_msg.position.z = goal_pose_camera[2, 3]

        # orientation을 quaternion으로 변환하여 설정
        quat = Rotation.from_matrix(goal_pose_camera[:3, :3]).as_quat()
        goal_pose_msg.orientation.x = quat[0]
        goal_pose_msg.orientation.y = quat[1]
        goal_pose_msg.orientation.z = quat[2]
        goal_pose_msg.orientation.w = quat[3]

        parent_frame = "camera_color_optical_frame"
        child_frame = "pvn_grasp_point"
        self.make_tf(goal_pose_msg, parent_frame, child_frame)  # TODO: stastic tf?

        # move with detection result
        # Z 방향이 파지하러 접근하는 방향
        # X 방향이 finger 1 방향
        self.move_from_detection_pvn(goal_pose_camera)


# hand_interface.move_pose(HandPose.HOME)
# hand_interface.move_pose(HandPose.READY)

# hand_interface.move_pose(HandPose.FULL_OPEN)
# hand_interface.move_pose(HandPose.FULL_CLOSE)

# hand_interface.move_pose(HandPose.BASIC_OPEN)
# hand_interface.move_pose(HandPose.BASIC_CLOSE)

# hand_interface.move_pose(HandPose.PINCH_OPEN)
# hand_interface.move_pose(HandPose.PINCH_CLOSE)

# hand_interface.move_pose(HandPose.WIDE_OPEN)
# hand_interface.move_pose(HandPose.WIDE_CLOSE)

# hand_interface.move_pose(HandPose.SCISSOR_OPEN)
# hand_interface.move_pose(HandPose.SCISSOR_CLOSE)

# hand_interface.move_pose(HandPose.SUCTION_PAD_BASIC)


def main(args=None):
    rclpy.init(args=args)
    indy_commander = IndyCommander()
    rclpy.spin(indy_commander)

    indy_commander.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()