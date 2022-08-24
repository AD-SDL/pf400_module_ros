#!/usr/bin/env python

# import rclpy
import os.path
import telnetlib
import threading
import json

import math
from operator import add
from time import sleep

# from sensor_msgs.msg import JointState

class TCSJointClient:

	commandLock = threading.Lock()
	# joint_state = JointState()

	def __init__(self, host, port, mode = 0, data_file_path = "robot_data.json", commands_file_path = "robot_commands.json", error_codes_path = "error_codes.json"):
		"""
        Description: 
        """

		print("Initializing connection...")
		self.host = host
		self.port = port
		self.mode = mode
		self.connection = None
		# robot_data, robot1, motion_profile, locations = self.load_robot_data(data_file_path)
		# self.robot_data = robot_data       
		# Default Motion Profile Paramiters. Using two profiles for faster and slower movements
		# self.motion_profile = motion_profile
		# Predefined locations for plate transferring oparetions
		# self.location_dictionary = locations
		# self.commands_list = self.load_robot_commands(commands_file_path)
		# self.error_codes = self.load_error_codes(error_codes_path)
		self.motion_profile = [
                {
                    "speed": 30,
                    "speed2": 0,
                    "acceleration": 100,
                    "deceleration": 100,
                    "accelramp": 0.1,
                    "decelramp": 0.1,
                    "inrange": 0,
                    "straight": -1
                },
                {
                    "speed": 50,
                    "speed2": 0,
                    "acceleration": 100,
                    "deceleration": 100,
                    "accelramp": 0.1,
                    "decelramp": 0.1,
                    "inrange": 60,
                    "straight": 0
                }]

		self.connect()
		self.init_connection_mode()
		
		self.axis_count = 6
		# self.joint_state.name = ["J{}".format(x + 1) for x in range(0, self.axis_count)] # Comment out for local testing
		print("Connection ready")


		self.gripper_open = 90.0
		self.gripper_closed = 79.0
		self.pf400_neutral = [399.992, -0.356, 181.867, 530.993, self.gripper_closed, 643.580]
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
		pass

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
		pass

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
		pass

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
		# TODO: Try exception will change top print the error but not kill the code

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


	def check_robot_state(self, wait:int = 0.1):
		"""
		Decription: Checks the robot state
		"""

		cmd = 'sysState'
		input_msg = 'Robot state'
		err_msg = 'Failed to check robot state:'

		out_msg = self.send_command(cmd)
		if "0 21" in out_msg:
			out_msg = "Robot intilized and in ready state"
		return out_msg


	def enable_power(self, wait:int = 0.1):
		"""
		Decription: Enables the power on the robot
		"""
		cmd = 'hp 1'


		out_msg = self.send_command(cmd)
		sleep(5)

		return out_msg

	def disable_power(self, wait:int = 0.1):
		"""
		Decription: Disables the power on the robot
		"""
		cmd = 'hp 0'
		ini_msg = 'Disabling power on the robot'
		err_msg = 'Failed disable_power:'

		out_msg = self.send_command(cmd)
		sleep(5)

		return out_msg

	def attach_robot(self, robot_id:str = "1", wait:int = 0.1):
		"""
		Decription: If there are multiple PF400 robots, chooses which robot will be programed attaches to the software. 
					If robot ID is not given it will attach the first robot.
		Parameters: 
				- robot_id: ID number of the robot
		"""
		cmd = "attach " + robot_id

		out_msg = self.send_command(cmd)
		sleep(5)

		return out_msg

		
	def home_robot(self, wait:int = 0.1):
		"""
		Decription: Homes robot joints. Homing takes around 15 seconds.
		"""
		cmd = 'home'
		ini_msg = 'Homing the robot'
		err_msg = 'Failed to home the robot: '

		out_msg = self.send_command(cmd)
		sleep(10)

		return out_msg

	# Create "profile section" apart from the "command section"
	def set_profile(self, wait:int = 0.1, profile_dict:dict = {"0":0}):
		"""
		Decription: Sets and saves the motion profiles (defined in robot data) to the robot. 
					If user defines a custom profile, this profile will saved onto motion profile 3 on the robot
		Parameters: 
				- profile_dict: Custom motion profile
		"""  
		if len(profile_dict) == 1:
			
			cmd = 'Profile 1'
			for key, value in self.motion_profile[0].items():
				cmd += ' ' + str(value)


			cmd2 = 'Profile 2'
			for key, value in self.motion_profile[1].items():
				cmd2 += ' ' + str(value)

			ini_msg = "Setting defult values to the motion profile 1"
			ini_msg2 = "Setting defult values to the motion profile 2"
			err_msg = 'Failed to set profile 1: '
			err_msg2 = 'Failed to set profile 2: '

			out_msg = self.send_command(cmd)
			out_msg2 = self.send_command(cmd2)



		elif len(profile_dict) == 8:

			ini_msg = "Setting new values to the motion profile 3"
			err_msg = 'Failed to set profile 1: '

			cmd = 'Profile 3'
			for key, value in profile_dict.items():
				cmd += ' ' + str(value)

			out_msg = self.send_command(cmd, ini_msg, err_msg, wait)
			
		else:
			raise Exception("Motion profile takes 8 arguments, {} where given".format(len(profile_dict)))

		return out_msg 

	def initialize_robot(self):
		"""
		Decription: Intilizes the robot by calling enable_power, attach_robot, home_robot, set_profile functions and 
					checks the robot state to find out if the initilization was successful
		"""

		# Enable power 
		power = self.enable_power()
		# Attach robot
		attach = self.attach_robot()
		# Home robot
		home = self.home_robot()
		# Set default motion profile
		profile = self.set_profile()
		# Check robots' current state
		rState =self.check_robot_state()

		if power[0].find('-') == -1 and attach[0].find('-') == -1 and home[0].find('-') == -1 and profile[0].find('-')== -1 :
			print("Robot initialization is successfully completed!")
		else:    
			print("Robot initialization failed!")

		return power + attach + profile + home + rState

	def check_general_state(self, wait:int = 0.1):
			"""
			Decription: Checks general state
			"""

			cmd1 = "hp"
			cmd2 = "attach"
			cmd3 = "sysState"

			power_msg = self.send_command(cmd1)
			power_msg = power_msg.split(" ")

			attach_msg = self.send_command(cmd2)
			attach_msg = attach_msg.split(" ")

			state_msg = self.send_command(cmd3)
			state_msg = state_msg.split(" ")

			power ,attach, state = 0, 0, 0

			if len(power_msg) == 1 or power_msg[0].find("-") != -1:
				power = -1
			if attach_msg[1].find("0") != -1 or attach_msg[0].find("-") != -1:
				attach = -1
			if state_msg[1].find("7") != -1 or state_msg[0].find("-") != -1:
				state = -1
			
			if power == -1 or attach == -1 or state == -1:
				return -1
			else: 
				return 0

	def set_robot_mode(self):
		"""
		Decription: Sets the robot to PC mode. This is needed to make sure the robot is properly communicating over the TCP socket. 
		"""
		
		cmd = 'mode 0'

		out_msg = self.send_command(cmd)
		sleep(5)
		return out_msg

	def force_initialize_robot(self):
		"""
		Decription: Repeats the initilzation until there are no errors and the robot is initilzed.
		"""

		self.set_robot_mode()
		# Check robot state & initilize
		if self.check_general_state() == -1:

			print("Robot is not intilized! Intilizing now...")
			output = self.initialize_robot()
			self.force_initialize_robot()

	def find_joint_states(self):
		"""
        Description: Locates the robot and returns the joint locations for all 6 joints.
        """
		states = self.send_command("wherej")
		joints = states.split(' ')
		joints = joints[1:] 
		return [float(x) for x in joints]

	def refresh_joint_state(self):
		"""
        Description: 
        """
		joint_array = self.find_joint_states()
		multipliers = [
			0.001,			# J1, Z
			math.pi / 180,	# J2, shoulder
			math.pi / 180,	# J3, elbow
			math.pi / 180,	# J4, wrist
			0.0005, 		# J5, gripper (urdf is 1/2 scale)
			0.0005, 		# J6, rail
		]
		# self.joint_state.raw_position = joint_array
		# self.joint_state.position = [state * multiplier for state, multiplier in zip(joint_array, multipliers)]

	def move_end_effector_neutral(self):
		"""
        Description: Move end effector to neutral position
        """
		# 231.788, -27.154, 313.011, 342.317, 0.0, 683.702
		current_joint_locations = self.find_joint_states()
		current_cartesian_coordinates = self.find_cartesian_coordinates()
		safe_y_distance = - 430
		if current_cartesian_coordinates[1] <= safe_y_distance:
			y_distance = safe_y_distance - current_cartesian_coordinates[1] 
			self.move_in_one_axis(1,0,y_distance,0)


		current_joint_locations[4] = self.gripper_closed
		current_joint_locations[3] = 530.993

		self.send_command(self.create_move_joint_command(current_joint_locations))

	def move_all_joints_neutral(self):
		"""
        Description: Move all joints to neutral position
        """

		self.move_end_effector_neutral()
		self.send_command(self.create_move_joint_command(self.pf400_neutral))

	def find_cartesian_coordinates(self):
		"""
        Description: This function finds the current cartesian coordinates and angles of the robot.
		Return: A float array with x/y/z yaw/pich/roll
        """

		coordinates = self.send_command("whereC")
		coordinates_list = coordinates.split(' ')
		coordinates_list = coordinates_list[1:-1]

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

	def move_in_one_axis_from_target(self, target_location, profile:int = 2, axis_x:int= 0,axis_y:int= 0, axis_z:int= 0):
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
		current_joint_state = self.find_joint_states()
		current_joint_state[5] = target_location[5]
		self.send_command(self.create_move_joint_command(current_joint_state))

		# Find the cartesian coordinates of the target joint states
		cartesian_coordinates = self.forward_kinematics(target_location)
		
		# Move en effector on the single axis
		cartesian_coordinates[0] += axis_x
		cartesian_coordinates[1] += axis_y
		cartesian_coordinates[2] += axis_z

		move_command = "MoveC "+ " " + str(profile) + " " + "".join(map(str, cartesian_coordinates))
		self.send_command(move_command)

	def move_in_one_axis(self,profile:int = 2, axis_x:int= 0,axis_y:int= 0, axis_z:int= 0):
		"""
		TODO: TRY THIS FUNCTION TO SEE IF END EFFECTOR MOVES ON A SINGLE AXIS PROPERLY

		Desciption: Moves the end effector on single axis with a goal movement in milimeters. 
		Paramiters:
			- target_location : Joint states of the target location
			- axis_x : Goal movement on x axis in mm
			- axis_y : Goal movement on y axis in mm
			- axis_z : Goal movement on z axis in mm
		"""

		# Find the cartesian coordinates of the target joint states
		cartesian_coordinates = self.find_cartesian_coordinates()
		
		# Move en effector on the single axis
		cartesian_coordinates[0] += axis_x
		cartesian_coordinates[1] += axis_y
		cartesian_coordinates[2] += axis_z

		move_command = "MoveC"+ " " + str(profile) + " " + " ".join(map(str, cartesian_coordinates))
		self.send_command(move_command)

	def create_move_cartesian_command(self, target_cartesian_coordinates, profile:int =2):

		move_command = "MoveC"+ " " + str(profile) + " " + " ".join(map(str, target_cartesian_coordinates))
		return move_command

	def create_move_joint_command(self, target_joint_locations, profile:int = 2, gripper_close: bool = False, gripper_open: bool = False):
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
		if gripper_close == True:
			target_joint_locations[4] = self.gripper_closed
		if gripper_open == True:
			target_joint_locations[4] = self.gripper_open

		move_command = "movej" + " " + str(profile) + " " + " ".join(map(str, target_joint_locations))

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
		self.send_command(self.create_move_joint_command(abovePos, fast_profile, False, True))
		self.send_command(self.create_move_joint_command(target_pose, slow_profile, False, True))
		# sleep(0.5)
		self.send_command(self.create_move_joint_command(target_pose, slow_profile, gripper_close=True, gripper_open= False))
		# sleep(0.5)
		self.send_command(self.create_move_joint_command(abovePos, slow_profile, True, False))
		sleep(1)
		self.move_all_joints_neutral()
		# TODO: USE BELOW MOVE_ONE_AXIS FUNCTIONS TO MOVE ABOVE AND FRONT OF THE EACH TARGET LOCATIONS
		# self.move_in_one_axis_from_target(target_pose, profile = 2, axis_x = 60, axis_y = 0, axis_z = 60)
		# self.move_in_one_axis_from_target(target_pose, profile = 1, axis_x = 0, axis_y = 0, axis_z = 60)

	def place_plate(self, target_pose):
		"""
        Description: 
        """
		slow_profile = 1
		fast_profile = 2

		# jointClosedPos = target_pose

		# target_pose[4] = self.gripper_open
		# jointClosedPos[4] = self.gripper_closed   

		abovePos = list(map(add, target_pose, self.above))
		# aboveClosedPos = list(map(add, jointClosedPos, self.above))

		# self.move_all_joints_neutral()
		self.send_command(self.create_move_joint_command(abovePos, fast_profile, True, False))
		self.send_command(self.create_move_joint_command(target_pose, slow_profile, True, False))
		self.send_command(self.create_move_joint_command(target_pose, slow_profile, False, True))
		self.send_command(self.create_move_joint_command(abovePos))
		sleep(1)
		self.move_all_joints_neutral()




	def transfer(self, location1, location2):
		"""
        Description: Plate transfer function that performs series of movements to pick and place the plates
		
        """
		self.force_initialize_robot()
		self.pick_plate(location1)
		self.place_plate(location2)

if __name__ == "__main__":

	target_joint_angles = "1 1 1 1 1 1"
	robot = TCSJointClient("192.168.50.50", 10100)
	robot.force_initialize_robot()
	loc1 = [262.550, 20.608, 119.290, 662.570, 126.0, 574.367]
	loc2 = [231.788, -27.154, 313.011, 342.317, 0.0, 683.702]
	# robot.initialize_robot()
	# robot.place_plate(loc2)
	robot.transfer(loc1, loc2)
	robot.transfer(loc2, loc1)
	# robot.initialize_robot()

	# robot.move_in_one_axis(1, 0, 0, -20)
	# robot.place_plate([262.550, 20.608, 119.290, 662.570, 126.0, 574.367])



