
import rospy
import time
import telnetlib

from TCSJointClient import TCSJointClient
from pf400_driver.pf400_driver import PF400
from sensor_msgs.msg import JointState
# from pa_viz_pf3400.srv import TCSService, TCSServiceResponse



# Default IP (overridden if launch file used)
HOST = "10.1.10.216"

PORT = "10000"
RATE = 20 #Hz

class Setup:

	tcs_client = None

	def run(self):

		# Get IP and port from parameters
		if rospy.has_param("TCS_IP"):
			HOST = rospy.get_param("TCS_IP")

		# Initialize ROS Node
		rospy.loginfo("initializing node")
		rospy.init_node("TCS", anonymous=False)

		# Initialize Publisher
		rospy.loginfo("initializing publisher")
		self.rate = rospy.Rate(RATE)
		self.jointStatePub = rospy.Publisher("joint_states", JointState, queue_size=100)

		# Start Telnet connection and initialize TCS
		self.tcs_client = TCSJointClient(HOST, PORT)

		# Loop and publish
		while not rospy.is_shutdown():
			try:
				self.tcs_client.RefreshJointState()
				self.jointStatePub.publish(self.tcs_client.joint_state)
				self.rate.sleep()

			except:
				pass



if __name__ == "__main__":
	Setup().run()