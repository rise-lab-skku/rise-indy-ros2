# import time
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped

from neuromeka import IndyDCP3

# DEG2RAD = 0.017453292519943295

"""
LINK 설명
================
indy_link0: Indy의 base_link
indy_flange: Indy에서 end-effector를 장착하는 flange. URDF에서 정의.
indy_flange_nrmkapi: IndyDCP 통신을 통해 받아오는 flange 위치. IndyDCP에서 정의. (F/T센서, 핸드아이 등 켈리브레이션 기준)
"""


class IndyJointStatePublisher(Node):
    def __init__(self):
        super().__init__("indy_joint_state_publisher")
        self.joint_state_publisher = self.create_publisher(JointState, "joint_states", 10)

        # connect to Indy
        self.declare_parameter("robot_ip", "Indy_RB2")
        self.robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        self.indy = IndyDCP3(self.robot_ip)

        # data
        self.num_joints = len(self.indy.get_control_data()["q"])
        self.joint_positions = np.zeros(self.num_joints)
        self.joint_names = [f"indy_joint{i}" for i in range(self.num_joints)]

        # msg
        self.joint_state_msg = JointState()
        self.joint_state_msg.header.frame_id = "indy_joint_state_publisher"
        self.joint_state_msg.name = self.joint_names
        self.tf_msg = TransformStamped()
        self.tf_msg.header.frame_id = "indy_link0"
        self.tf_msg.child_frame_id = "indy_flange_nrmkapi"

        # timer
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.update_data)  # 20 Hz

    def update_data(self):
        # Read data
        self.joint_positions = np.array(self.indy.get_control_data()["q"])
        task_pose = self.indy.get_control_data()["p"]
        t = self.get_clock().now().to_msg()

        # Publish data
        self.joint_positions = np.deg2rad(self.joint_positions)
        self.publish_joint_states(t)

        # convert millimeter to meter
        trans = np.array(task_pose[:3])
        trans *= 0.001
        quat = R.from_euler("xyz", task_pose[3:6], degrees=True).as_quat()
        self.publish_tf(t, trans, quat)

    def publish_joint_states(self, t):
        self.joint_state_msg.header.stamp = t
        self.joint_state_msg.position = self.joint_positions.tolist()
        self.joint_state_publisher.publish(self.joint_state_msg)

    def publish_tf(self, t, trans, quat):
        self.tf_msg.header.stamp = t
        self.tf_msg.transform.translation.x = trans[0]
        self.tf_msg.transform.translation.y = trans[1]
        self.tf_msg.transform.translation.z = trans[2]
        self.tf_msg.transform.rotation.x = quat[0]
        self.tf_msg.transform.rotation.y = quat[1]
        self.tf_msg.transform.rotation.z = quat[2]
        self.tf_msg.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(self.tf_msg)


def main(args=None):
    rclpy.init(args=args)
    joint_controller = IndyJointStatePublisher()

    rclpy.spin(joint_controller)

    joint_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
