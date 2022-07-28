#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile


from TCSJointClient import TCSJointClient

class PF_JOINTS(Node):
	def __init__(self, HOST, PORT):
		super().__init__('pf400_joint_publisher')

		# Start Telnet connection and initialize TCS
		self.HOST = HOST
		self.PORT = PORT
		timer_period = 0.1  # seconds
		self.tcs_client = TCSJointClient(self.HOST, self.PORT)
		qos_profile = QoSProfile(depth=100)
		self.jointStatePub = self.create_publisher(JointState, "joint_states",  qos_profile)
		self.timer = self.create_timer(timer_period, self.joint_callback)

	def joint_callback(self):
		try:
			self.tcs_client.RefreshJointState()
			self.jointStatePub.publish(self.tcs_client.joint_state)
			print(self.tcs_client.joint_state.position)
		except:
			pass

def main(args=None):

	# Default IP (overridden if launch file used)
	HOST = "192.168.50.50"

	PORT = "10100"

	rclpy.init(args=args)

	pf400_joint_publisher = PF_JOINTS(HOST,PORT)

	rclpy.spin(pf400_joint_publisher)

	pf400_joint_publisher.destroy_node()
	rclpy.shutdown()




if __name__ == '__main__':
	main()