#!/usr/bin/env python

import rclpy
import os.path
import telnetlib
import threading
import json

import math
from operator import add
from time import sleep

from sensor_msgs.msg import JointState

class TCSJointClient:

	commandLock = threading.Lock()
	joint_state = JointState()

	def __init__(self, host, port, mode = 0, data_file_path = "robot_data.json", commands_file_path = "robot_commands.json", error_codes_path = "error_codes.json"):
		"""
        Description: 
        """

		print("Initializing connection...")
		self.host = host
		self.port = port
		self.mode = mode
		self.connection = None
		robot_data, robot1, motion_profile, locations = self.load_robot_data(data_file_path)
		self.robot_data = robot_data       
		# Default Motion Profile Paramiters. Using two profiles for faster and slower movements
		self.motion_profile = motion_profile
		# Predefined locations for plate transferring oparetions
		self.location_dictionary = locations
		self.commands_list = self.load_robot_commands(commands_file_path)
		self.error_codes = self.load_error_codes(error_codes_path)

		self.connect()
		self.init_connection_mode()
		
		self.axis_count = 6
		self.joint_state.name = ["J{}".format(x + 1) for x in range(0, self.axis_count)] # Comment out for local testing
		print("Connection ready")


		self.gripper_open = 90.0
		self.gripper_closed = 79.0
		self.pf400_neutral = [399.992, -0.356, 181.867, 530.993, self.gripper_open, 643.580]
		self.above = [60.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	def load_robot_data(self, data_file_path):
		"""
		Decription: Loads the robot identification/motion profile/location data as dictionaries 
		Parameters: 
				- data_file_path: Path to data file
		"""
		# Setting parent file directory 
		current_directory = os.path.dirname(__file__)
		parent_directory = os.path.split(current_directory)[0] 
		file_path = os.path.join(parent_directory + '/utils/'+ data_file_path)

		# load json file
		with open(file_path) as f:
			data = json.load(f)

		f.close()

		return data, data["robot_data"][0], data["robot_data"][0]["motion_profile"],data["robot_data"][0]["locations"][0]


	def load_robot_commands(self, commands_file_path):
		"""
		Decription: Loads the available command list as a list
		Parameters: 
				- commands_file_path: Path to command list file
		"""
		# Setting parent file directory 
		current_directory = os.path.dirname(__file__)
		parent_directory = os.path.split(current_directory)[0] 
		file_path = os.path.join(parent_directory + '/utils/'+ commands_file_path)

		# load json file
		with open(file_path) as f:
			data = json.load(f)

		f.close()
		return data["Commands_List"]

	def load_error_codes(self, error_codes_path):
		"""
		Decription: Loads the robot error codes data as a dictionary
		Parameters: 
				- error_codes_path: Path to error codes data file
		"""
		# Setting parent file directory 
		current_directory = os.path.dirname(__file__)
		parent_directory = os.path.split(current_directory)[0] 
		file_path = os.path.join(parent_directory + '/utils/'+ error_codes_path)

		# load json file
		with open(file_path) as f:
			data = json.load(f)

		f.close()
		return data["Error_Codes"]

	def connect(self):
		"""
        Description: 
        """
		try:
			self.connection = telnetlib.Telnet(self.host, self.port, 5)
		except:
			raise Exception("Could not establish connection")

	def disconnect(self):
		"""
        Description: 
        """
		self.connection.close()

	def send_command(self, command):
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

	def init_connection_mode(self):
		"""
        Description: 
        """
		if not self.connection:
			self.Connect()
		if self.mode == 0:
			# Set TCS to nonverbose
			self.send_command("mode 0")
		else:
			# Set TCS to verbose
			self.send_command("mode 1")
		self.send_command("selectrobot 1")

	def initilize_robot(self):
		pass

	def get_joint_data(self):
		"""
        Description: Locates the robot and returns the joint locations for all 6 joints.
        """
		states = self.send_command("wherej")
		joints = states.split(' ')
		joints = joints[1:] # WHY THE FIRST ELEMENT SKIPPED?
		return [float(x) for x in joints]

	def refresh_joint_state(self):
		"""
        Description: 
        """
		joint_array = self.get_joint_data()
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
        Description: Move end effector to neutral position
        """

		current_joint_locations = self.get_joint_data()
		current_joint_locations[3] = 530.993
		self.send_command(self.create_move_command(current_joint_locations))

	def move_all_joints_neutral(self):
		"""
        Description: Move all joints to neutral position
        """

		self.move_end_effector_neutral()
		self.send_command(self.create_move_command(self.pf400_neutral))

	def find_cartesian_coordinates(self):
		"""
        Description: This function finds the current cartesian coordinates and angles of the robot.
		Return: A float array with x/y/z yaw/pich/roll
        """

		coordinates = self.send_command("whereC")
		coordinates_list = coordinates.split(' ')
		coordinates_list = coordinates_list[1:] # WHY THE FIRST ELEMENT SKIPPED?

		return [float(x) for x in coordinates_list]

	def forward_kinematics(self, joint_states):
		"""
		Desciption: Calculates the forward kinematics for a given array of joint_states. 
		Paramiters:
			- joint_states : 6 joint states of the target location/
		Return:
			- cartesian_coordinates: Returns the calculated cartesian coordinates of the given joint states
		"""
		cartesian_coordinates = self.find_cartesian_coordinates()
		shoulder_lenght = 225
		elbow_lenght = 210

		# Convert angles to radians
		shoulder_angle = joint_states[1]*math.pi/180 #Joint 2 
		elbow_angle = joint_states[2]*math.pi/180 #Joint 3

		x = shoulder_lenght*math.cos(shoulder_angle) + elbow_lenght*math.cos(shoulder_angle+elbow_angle)
		y = shoulder_lenght*math.sin(shoulder_angle) + elbow_lenght*math.sin(shoulder_angle+elbow_angle)
		z = joint_states[0]

		cartesian_coordinates[0] = x
		cartesian_coordinates[1] = y
		cartesian_coordinates[2] = z
		print(x, y, z)

		return cartesian_coordinates

	def move_in_one_axis(self, target_location, profile:int = 2, axis_x:int= 0,axis_y:int= 0, axis_z:int= 0):
		"""
		TODO: TRY THIS FUNCTION TO SEE IF END EFFECTOR MOVES ON A SINGLE AXIS PROPERLY

		Desciption: Moves the end effector on single axis with a goal movement in milimeters. 
		Paramiters:
			- target_location : Joint states of the target location
			- axis_x : Goal movement on x axis in mm
			- axis_y : Goal movement on y axis in mm
			- axis_z : Goal movement on z axis in mm
		"""
		# First move robot on linear rail
		current_joint_state = self.get_joint_data()
		current_joint_state[5] = target_location[5]
		self.send_command(self.create_move_command(current_joint_state))

		# Find the cartesian coordinates of the target joint states
		cartesian_coordinates = self.forward_kinematics(target_location)
		
		# Move en effector on the single axis
		cartesian_coordinates[0] += axis_x
		cartesian_coordinates[1] += axis_y
		cartesian_coordinates[2] += axis_z

		move_command = "MoveC "+ " " + str(profile) + " " + "".join(map(str, cartesian_coordinates))
		self.send_command(move_command)

	def create_move_command(self, target_joint_locations, profile:int = 2, gripper_close: bool = False, gripper_open: bool = False):
		"""
		Description: Creates the movement commands with the given robot_location, profile, gripper closed and gripper open info
		Parameters:
				- target_location: Which location the PF400 will move.
				- profile: Motion profile ID.
				- gripper_close: If set to TRUE, gripper is closed. If set to FALSE, gripper position will remain same as the previous location. 
				- gripper_open: If set to TRUE, gripper is opened. If set to FALSE, gripper position will remain same as the previous location.
		Return: Returns the created movement command in string format
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
		
	def pick_plate(self, target_pose):
		"""
        Description: 
        """
		#------
		slow_profile = 1
		fast_profile = 2
		jointClosedPos = target_pose

		# target_pose[4] = self.gripper_open
		# jointClosedPos[4] = self.gripper_closed   


		#------

		abovePos = list(map(add, target_pose, self.above))
		aboveClosedPos = list(map(add, jointClosedPos, self.above))
		#------

		# move_neutral
		# move_front 
        # above_plate 
        # approach_plate 
        # pick_up_plate 
        # above_with_plate
        # front_with_plate 

		self.move_all_joints_neutral()
		self.send_command(self.create_move_command(abovePos, fast_profile, False, True))
		self.send_command(self.create_move_command(target_pose, slow_profile, False, True))
		sleep(0.5)
		self.send_command(self.create_move_command(target_pose, slow_profile, True, False))
		sleep(0.5)
		self.send_command(self.create_move_command(abovePos, slow_profile, True, False))
		self.move_all_joints_neutral()
		# TODO: USE BELOW MOVE_ONE_AXIS FUNCTIONS TO MOVE ABOVE AND FRONT OF THE EACH TARGET LOCATIONS
		# self.move_in_one_axis(target_pose, profile = 2, axis_x = 60, axis_y = 0, axis_z = 60)
		# self.move_in_one_axis(target_pose, profile = 1, axis_x = 0, axis_y = 0, axis_z = 60)

	def place_plate(self, target_pose):
		"""
        Description: 
        """
		slow_profile = 1
		fast_profile = 2

		jointClosedPos = target_pose

		target_pose[4] = self.gripper_open
		jointClosedPos[4] = self.gripper_closed   

		abovePos = list(map(add, target_pose, self.above))
		aboveClosedPos = list(map(add, jointClosedPos, self.above))

		self.move_all_joints_neutral()
		self.send_command(self.create_move_command(abovePos, slow_profile, True, False))
		self.send_command(self.create_move_command(target_pose, slow_profile, True, False))
		self.send_command(self.create_move_command(target_pose, slow_profile, False, True))
		self.send_command(self.create_move_command(abovePos))



	def transfer(self, location1, location2):
		"""
        Description: 
		
        """
		self.pick_plate(location1)
		self.place_plate(location2)

if __name__ == "__main__":
	target_joint_angles = "1 1 1 1 1 1"

