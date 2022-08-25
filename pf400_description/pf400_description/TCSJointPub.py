#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster, TransformStamped

from ...pf400_driver.pf400_driver.pf400_driver import PF400

class PF_JOINTS(Node):
	def __init__(self, HOST, PORT):
		super().__init__('pf400_joint_pub')

		# Start Telnet connection and initialize TCS
		self.HOST = HOST
		self.PORT = PORT
		timer_period = 0.1  # seconds
		self.tcs_client = PF400(self.HOST, self.PORT)
		qos_profile = QoSProfile(depth=100)
		self.jointStatePub = self.create_publisher(JointState, "joint_states",  qos_profile)
		# self.timer = self.create_timer(timer_period, self.joint_callback)

		##########################  Testing Stuff

		loop_rate = self.create_rate(30)

		self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

		odom_trans = TransformStamped()
		odom_trans.header.frame_id = 'world'
		odom_trans.child_frame_id = 'z_collumn'
		joint_state = JointState()

		try:
			while rclpy.ok():
				rclpy.spin_once(self)
				# update joint_state
				now = self.get_clock().now()
				joint_state.header.stamp = now.to_msg()
				joint_state.position = self.tcs_client.joint_state.position

				odom_trans.header.stamp = now.to_msg()

				self.jointStatePub.publish(joint_state)
				self.broadcaster.sendTransform(odom_trans)
				loop_rate.sleep()

		except:
			pass


	# def joint_callback(self):
	# 	try:
	# 		self.tcs_client.RefreshJointState()
	# 		self.jointStatePub.publish(self.tcs_client.joint_state)
	# 		print(self.tcs_client.joint_state.position)
	# 	except:
	# 		pass

def main(args=None):

	# Default IP (overridden if launch file used)
	HOST = "192.168.50.50"

	PORT = "10100"

	rclpy.init(args=args)

	pf400_joint_publisher = PF_JOINTS(HOST, PORT)

	rclpy.spin(pf400_joint_publisher)

	pf400_joint_publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()