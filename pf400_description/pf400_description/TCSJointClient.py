#!/usr/bin/env python

import rospy
import telnetlib
import threading
import time
import math

from sensor_msgs.msg import JointState
from pa_viz_pf3400.srv import TCSService, TCSServiceResponse

class TCSJointClient:

	commandLock = threading.Lock()
	joint_state = JointState()

	def __init__(self, host, port):
		self.host = host
		self.port = port
		self.connection = None
		self.axis_count = 5
		self.joint_state.name = ["J{}".format(x + 1) for x in range(0, self.axis_count)]
		self.Connect()
		self.InitConnectionMode()
		self.RefreshJointState()

	def Connect(self):
		self.connection = telnetlib.Telnet(self.host, self.port)

	def Disconnect(self):
		self.connection.close()

	def InitConnectionMode(self):
		self.commandLock.acquire()
		try:	
			if not self.connection:
				self.Connect()

			# Set TCS to machine-readable form
			self.connection.write("mode 0\n")
			self.connection.read_until("\n")
			self.connection.write("selectrobot 1\n")
			self.connection.read_until("\n")

		finally:
			self.commandLock.release()

	def SendCommandJoints(self, command):
		self.commandLock.acquire()
		try:
			if not self.connection:
				self.Connect()
			self.connection.write(command + "\n")
			line = self.connection.read_until("\n").rstrip()
			response = line.split(" ", 1)
			jangles = response[1] if len(response) > 1 else ""
			return TCSServiceResponse(jangles)

		finally:
			self.commandLock.release()

	def RefreshJointState(self):
		try:
			joint_array = self.GetJointData()
			multipliers = [
				0.001,			# J1, Z
				math.pi / 180,	# J2, shoulder
				math.pi / 180,	# J3, elbow
				math.pi / 180,	# J4, wrist
				0.0005, 		# J5, gripper (urdf is 1/2 scale)
			]
			self.joint_state.header.stamp = rospy.Time.now()
			self.joint_state.position = \
				[state * multiplier for state, multiplier in zip(joint_array, multipliers)]

		except Exception as err:
			rospy.logerr(err)
			time.sleep(5)

	def GetJointData(self):
		states = self.SendCommandJoints("wherej").output.split(" ")
		return [float(x) for x in states]