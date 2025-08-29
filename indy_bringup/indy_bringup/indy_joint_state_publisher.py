import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import sys
sys.path.append("./")
from neuromeka import IndyDCP3
import numpy as np

DEG2RAD = 0.017453292519943295

class IndyJointStatePublisher(Node):
    def __init__(self):
        super().__init__('indy_joint_state_publisher')
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # connect to Indy
        self.declare_parameter("robot_ip", "Nsquare")
        self.robot_ip = (
            self.get_parameter("robot_ip").get_parameter_value().string_value
        )
        self.indy = IndyDCP3(self.robot_ip)

        self.joint_num = len(self.indy.get_control_data()['q'])
        self.joint_positions = [0.0] * (self.joint_num)
        self.joint_names = [f'indy_joint{i}' for i in range(self.joint_num)] # + ['indy_flange']
        self.timer = self.create_timer(0.05, self.publish_joint_states)

    def publish_joint_states(self):
        self.update_joint_positions()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'indy_joint_state_publisher'
        msg.name = self.joint_names
        msg.position = self.joint_positions

        self.joint_state_publisher.publish(msg)
    
    def update_joint_positions(self):
        self.joint_positions = self.indy.get_control_data()['q']
        self.joint_positions = [joint_deg * DEG2RAD for joint_deg in self.joint_positions] # + [0.0]

def main(args=None):
    rclpy.init(args=args)
    joint_controller = IndyJointStatePublisher()

    rclpy.spin(joint_controller)

    joint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
