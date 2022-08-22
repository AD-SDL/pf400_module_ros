#!/usr/bin/env python

# import rclpy
import telnetlib
import threading
import math
from operator import add
from time import sleep
from unicodedata import name

# from sensor_msgs.msg import JointState

class TCSJointClient:

	commandLock = threading.Lock()
	# joint_state = JointState()

	def __init__(self, host, port, mode = 0):
		"""
        Description: 
        """

		print("Initializing connection...")
		self.host = host
		self.port = port
		self.mode = mode
		self.connection = None
		self.Connect()
		self.InitConnectionMode()
		self.axis_count = 6
		# self.joint_state.name = ["J{}".format(x + 1) for x in range(0, self.axis_count)] # Comment out for local testing
		print("Connection ready")


		self.gripper_open = 90.0
		self.gripper_closed = 79.0
		self.pf400_neutral = [399.992, -0.356, 181.867, 530.993, self.gripper_open, 643.580]
		self.above = [60.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	def load_robot_data(self, data_file_path):
		"""
        Description: 
        """
		pass #TODO

	def load_robot_commands(self, commands_file_path):
		"""
        Description: 
        """
		pass #TODO

	def Connect(self):
		"""
        Description: 
        """
		try:
			self.connection = telnetlib.Telnet(self.host, self.port, 5)
		except:
			raise Exception("Could not establish connection")

	def Disconnect(self):
		"""
        Description: 
        """
		self.connection.close()

	def SendCommand(self, command):
		"""
        Description: 
        """
		self.commandLock.acquire()
		try:
			if not self.connection:
				self.Connect()		
			print(">> " + command)
			self.connection.write((command.encode("ascii") + b"\n"))
			if self.mode == 1:
				response1 = self.connection.read_until(b"\r\n").rstrip().decode("ascii")
				if response1 != "" and response1[0] == "-":
					raise Exception("TCS error: " + response1)
				print("<< "+ response1)
				return response1		
			else:
				response2 = self.connection.read_until(b"\r\n").rstrip().decode("ascii")
				if response2 != "" and response2[0] == "-":
					raise Exception("TCS error: " + response2)
				print("<< "+ response2)
				return response2		
			# TODO:Redo the error return code detection in the response massage by reading from error code list
		finally:
			self.commandLock.release()

	def InitConnectionMode(self):
		"""
        Description: 
        """
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
		"""
        Description: Locates the robot and returns the joint locations for all 6 joints.
        """
		states = self.SendCommand("wherej")
		joints = states.split(' ')
		joints = joints[1:]
		return [float(x) for x in joints]

	def RefreshJointState(self):
		"""
        Description: 
        """
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

	def move_end_effector_neutral(self):
		"""
        Description: 
        """
		pass #TODO
	
	def move_all_joints_neutral(self):
		"""
        Description: 
        """
		pass

	def find_cartesian_coordinates(self):
		# Obtain current robot cartesian coordinates
		# Return an int array with x/y/z yaw/pich/roll
		pass

	def move_in_one_axis(self, profile:int = 2, axis:str= "x", value:int = 0):
		"""
		TODO: TRY THIS FUNCTION TO SEE IF END EFFECTOR MOVES ON A SINGLE AXIS PROPERLY
		"""
		cartesian_coordinates = self.find_cartesian_coordinates()
		if axis == "x":
			cartesian_coordinates[0] += value
		elif axis == "y":
			cartesian_coordinates[1] += value
		elif axis == "z":
			cartesian_coordinates[2] += value

		move_command = "MoveC "+ " " + str(profile) + " " + "".join(map(str, cartesian_coordinates))
		self.SendCommand(move_command)

	def create_move_command(self, target_joint_locations, profile:int = 2, gripper_close: bool = False, gripper_open: bool = False):
		"""
		Description: Creates the movement commands with the given robot_location, profile, gripper closed and gripper open info
		Parameters:
				- target_location: Which location the PF400 will move.
				- profile: Motion profile ID.
				- gripper_close: If set to TRUE, gripper is closed. If set to FALSE, gripper position will remain same as the previous location. 
				- gripper_open: If set to TRUE, gripper is opened. If set to FALSE, gripper position will remain same as the previous location.
		"""

		# Checking unpermitted gripper command
		if gripper_close == True and gripper_open == True:
			raise Exception("Gripper cannot be open and close at the same time!")
			
		# Setting the gripper location to open or close. If there is no gripper position passed in, target_joint_locations will be used.
		if gripper_close == "True":
			target_joint_locations[4] = self.gripper_closed
		elif gripper_open == "True":
			target_joint_locations[4] = self.gripper_open

		move_command = "movej" + " " + str(profile) + " " + "".join(map(str, target_joint_locations))

		return move_command
		


	def pick_plate(self, jointPos):
		"""
        Description: 
        """
		#------
		slow_profile = 1
		fast_profile = 2
		profile = 2
		jointClosedPos = jointPos

		# jointPos[4] = self.gripper_open
		# jointClosedPos[4] = self.gripper_closed   
		#------

		abovePos = list(map(add, jointPos, self.above))
		aboveClosedPos = list(map(add, jointClosedPos, self.above))
		#------
		
		# cmd = "movej" + " " + str(profile) + " " + " ".join(map(str, self.pf400_neutral))  # Moves pf400 to neutral position
		
		cmd =self.create_move_command(self.pf400_neutral)
		self.SendCommand(cmd)

		# cmd = "movej" + " " + str(profile) + " " + " ".join(map(str, abovePos))  # Moves pf400 to neutral position
		
		cmd = self.create_move_command(abovePos, slow_profile, False, True)
		self.SendCommand(cmd)

		# cmd = "movej" + " " + str(profile) + " " + " ".join(map(str, jointPos))  # Moves pf400 to neutral position
        # cmd = "movej" + " " + str(profile) + " " + str(self.jointPos[0]) + " " + str(self.jointPos[1]) + " " + str(self.jointPos[2]) + " " + str(self.jointPos[3])+ " " + str(self.gripper_open) + " " + str(self.jointPos[5])        
		
		cmd = self.create_move_command(jointPos, slow_profile, False, True)
		self.SendCommand(cmd)
		sleep(0.5)

		# cmd = "movej" + " " + str(profile) + " " + " ".join(map(str, jointClosedPos))  # Moves pf400 to neutral position
		
		cmd = self.create_move_command(jointPos, slow_profile, True, False)
		self.SendCommand(cmd)
		sleep(0.5)

		# cmd = "movej" + " " + str(profile) + " " + " ".join(map(str, aboveClosedPos))  # Moves pf400 to neutral position
		cmd = self.create_move_command(abovePos, slow_profile, True, False)
		self.SendCommand(cmd)

		# cmd = "movej" + " " + str(profile) + " " + " ".join(map(str, self.pf400_neutral))  # Moves pf400 to neutral position
		# self.SendCommand(cmd)


	def place_plate(self, jointPos):
		"""
        Description: 
        """
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
		"""
        Description: 
		
        """
		self.pick_plate(location1)
		self.place_plate(location2)

if __name__ == "__main__":
	target_joint_angles = "1 1 1 1 1 1"
	# profile = 2
	# cmd = "movej" + " " + str(profile) + " " + "".join(map(str, target_joint_angles))  # Moves pf400 to neutral position
	# print(cmd)
	# robot = TCSJointClient("127.0.0.1", "8080")
	# robot.pick(target_joint_angles)