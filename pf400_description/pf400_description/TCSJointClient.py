#!/usr/bin/env python

import rclpy
import telnetlib
import threading
import math
from operator import add
from time import sleep

from sensor_msgs.msg import JointState

class TCSJointClient:

	commandLock = threading.Lock()
	joint_state = JointState()

	def __init__(self, host, port, mode = 0):
		print("Initializing connection...")
		self.host = host
		self.port = port
		self.mode = mode
		self.connection = None
		self.Connect()
		self.InitConnectionMode()
		self.axis_count = 6
		self.joint_state.name = ["J{}".format(x + 1) for x in range(0, self.axis_count)]
		print("Connection ready")


		self.gripper_open = 90.0
		self.gripper_closed = 79.0

		self.pf400_neutral = [399.992, -0.356, 181.867, 530.993, self.gripper_open, 643.580]

		self.above = [60.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	
	def Connect(self):
		try:
			self.connection = telnetlib.Telnet(self.host, self.port, 5)
		except:
			raise Exception("Could not establish connection")

	def Disconnect(self):
		self.connection.close()

	def SendCommand(self, command):
		self.commandLock.acquire()
		try:
			if not self.connection:
				self.Connect()		
			print(">> " + command)
			self.connection.write((command.encode("ascii") + b"\n"))
			if self.mode == 1:
				response1 = self.connection.read_until(b"\r\n").rstrip().decode("ascii")
			response2 = self.connection.read_until(b"\r\n").rstrip().decode("ascii")
			if response2 != "" and response2[0] == "-":
				raise Exception("TCS error: " + response2)
			print("<< "+ response2)
			return response2		
		finally:
			self.commandLock.release()

	def InitConnectionMode(self):
		if not self.connection:
			self.Connect()
		if self.mode == 0:
			# Set TCS to nonverbose
			self.SendCommand("mode 0")
		else:
			# Set TCS to verbose
			self.SendCommand("mode 1")
		self.SendCommand("selectrobot 1")

	def GetJointData(self):
		states = self.SendCommand("wherej")
		joints = states.split(' ')
		joints = joints[1:]
		return [float(x) for x in joints]

	def RefreshJointState(self):
		joint_array = self.GetJointData()
		multipliers = [
			0.001,			# J1, Z
			math.pi / 180,	# J2, shoulder
			math.pi / 180,	# J3, elbow
			math.pi / 180,	# J4, wrist
			0.0005, 		# J5, gripper (urdf is 1/2 scale)
			0.0005, 		# J6, rail
		]
		# self.joint_state.raw_position = joint_array
		self.joint_state.position = [state * multiplier for state, multiplier in zip(joint_array, multipliers)]


	def pick(self, jointPos):

		profile = 2
		jointClosedPos = jointPos

		jointPos[4] = self.gripper_open
		jointClosedPos[4] = self.gripper_closed   

		abovePos = list(map(add, jointPos, self.above))
		aboveClosedPos = list(map(add, jointClosedPos, self.above))

		cmd = "movej" + " " + str(profile) + " " + " ".join(map(str, self.pf400_neutral))  # Moves pf400 to neutral position
		self.SendCommand(cmd)


		cmd = "movej" + " " + str(profile) + " " + " ".join(map(str, abovePos))  # Moves pf400 to neutral position
		self.SendCommand(cmd)

		cmd = "movej" + " " + str(profile) + " " + " ".join(map(str, jointPos))  # Moves pf400 to neutral position
        # cmd = "movej" + " " + str(profile) + " " + str(self.jointPos[0]) + " " + str(self.jointPos[1]) + " " + str(self.jointPos[2]) + " " + str(self.jointPos[3])+ " " + str(self.gripper_open) + " " + str(self.jointPos[5])        

		self.SendCommand(cmd)

		sleep(0.5)

		cmd = "movej" + " " + str(profile) + " " + " ".join(map(str, jointClosedPos))  # Moves pf400 to neutral position

		self.SendCommand(cmd)

		sleep(0.5)

		cmd = "movej" + " " + str(profile) + " " + " ".join(map(str, aboveClosedPos))  # Moves pf400 to neutral position

		self.SendCommand(cmd)

		# cmd = "movej" + " " + str(profile) + " " + " ".join(map(str, self.pf400_neutral))  # Moves pf400 to neutral position
		# self.SendCommand(cmd)


	def place(self, jointPos):

		profile = 2
		jointClosedPos = jointPos

		jointPos[4] = self.gripper_open
		jointClosedPos[4] = self.gripper_closed   

		abovePos = list(map(add, jointPos, self.above))
		aboveClosedPos = list(map(add, jointClosedPos, self.above))

		cmd = "movej" + " " + str(profile) + " " + " ".join(map(str, self.pf400_neutral))  # Moves pf400 to neutral position
		self.SendCommand(cmd)


		cmd = "movej" + " " + str(profile) + " " + " ".join(map(str, aboveClosedPos))  # Moves pf400 to neutral position
		self.SendCommand(cmd)

		cmd = "movej" + " " + str(profile) + " " + " ".join(map(str, jointClosedPos))  # Moves pf400 to neutral position
        # cmd = "movej" + " " + str(profile) + " " + str(self.jointPos[0]) + " " + str(self.jointPos[1]) + " " + str(self.jointPos[2]) + " " + str(self.jointPos[3])+ " " + str(self.gripper_open) + " " + str(self.jointPos[5])        

		self.SendCommand(cmd)

		sleep(0.5)

		cmd = "movej" + " " + str(profile) + " " + " ".join(map(str, jointPos))  # Moves pf400 to neutral position

		self.SendCommand(cmd)

		sleep(0.5)

		cmd = "movej" + " " + str(profile) + " " + " ".join(map(str, abovePos))  # Moves pf400 to neutral position

		self.SendCommand(cmd)

		cmd = "movej" + " " + str(profile) + " " + " ".join(map(str, self.pf400_neutral))  # Moves pf400 to neutral position
		self.SendCommand(cmd)

	def transfer(self, location1, location2):

		self.pick(location1)
		self.place(location2)

