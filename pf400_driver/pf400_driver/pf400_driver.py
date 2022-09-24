#!/usr/bin/env python3

import rclpy
import profile
import telnetlib
import threading

import math
from operator import add
from time import sleep

from pf400_driver.motion_profiles import motion_profiles
from pf400_driver.error_codes import error_codes

class PF400():
	commandLock = threading.Lock()

	def __init__(self, host, port, mode = 0):
		"""
        Description: 
        """

		print("Initializing connection...")
		self.host = host
		self.port = port
		self.mode = mode
		self.connection = None

		# Error code list of the PF400
		self.error_codes = error_codes

		# Default Motion Profile Paramiters. Using two profiles for faster and slower movements
		self.motion_profiles = motion_profiles

		##Robot State
		self.power_state = "0"
		self.attach_state = "0"
		self.home_state = "1"
		self.gripper_state = " "
		self.initialization_state = "0"
		self.robot_state = "Normal"

		self.connect()
		self.init_connection_mode()
		self.force_initialize_robot()

		##gripper vars
		self.gripper_open_state = 95.0
		self.gripper_closed_state = 77.0
		self.gripper_safe_height = 10.0
		self.gripper_state = self.find_gripper_state()

		##arm vars
		self.neutral_joints = [400.0, 1.400, 177.101, 536.757, self.gripper_closed_state, 0.0]	
		self.module_left_dist = -420.0
		self.module_right_dist = 220.0


		##sample vars
		self.sample_above_height = 60.0
		self.above = [self.sample_above_height,0,0,0,0,0]
		self.y_recoil = 300.0	

	def connect(self):
		"""
        """
		try:
			self.connection = telnetlib.Telnet(self.host, self.port, 5)
		except:
			raise Exception("Could not establish connection")

	def disconnect(self):
		"""
        """
		self.connection.close()

	def send_command(self, command):
		"""
        """

		self.commandLock.acquire()
		
		try:
			if not self.connection:
				self.Connect()	
				
			# if command.split()[0] == "movej" or command.split()[0] == "movec" or command.split()[0] == "wherej" or command.split()[0] == "wherec":	
			if self.robot_movement_state() > 1:
				print("Waiting for robot movement to end before sending the new command")
				while self.robot_movement_state() > 1:
					dummy_loop = 0

			print(">> " + command)
			self.connection.write((command.encode("ascii") + b"\n"))
			response = self.connection.read_until(b"\r\n").rstrip().decode("ascii")
			if response != "" and response in self.error_codes:
				self.handle_error_output(response)
			else:
				print("<< "+ response)
				self.robot_state = "Normal"

			return response		

		finally:
			self.commandLock.release()

	def robot_movement_state(self):
		"""Checks the movement state of the robot
		States: 0 = Power off
				1 = Stopping
				2 = Accelarating
				3 = Deccelarating	
		"""

		self.connection.write(("state".encode("ascii") + b"\n"))
	
		movement_state = self.connection.read_until(b"\r\n").rstrip().decode("ascii")

		if movement_state != "" and movement_state in self.error_codes:
			self.handle_error_output(movement_state)
		else:
			movement_state = int(movement_state.split()[1])
			self.robot_state = "Normal"

		return movement_state		

	def init_connection_mode(self):
		"""
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

	def handle_error_output(self, output):
		"""Handles the error message output
		"""
		if output in self.error_codes:
			print("<< " + self.error_codes[output])
		else:
			print("<< TCS Unknown error: " + output)

		self.robot_state = "ERROR"


	def check_robot_state(self, wait:int = 0.1):
		"""
		Decription: Checks the robot state
		"""

		out_msg = self.send_command('sysState')
		if "0 21" in out_msg:
			out_msg = "Robot intilized and in ready state"
		return out_msg


	def enable_power(self, wait:int = 0.1):
		"""
		Decription: Enables the power on the robot
		"""

		out_msg = self.send_command('hp 1')
		return out_msg

	def disable_power(self, wait:int = 0.1):
		"""
		Decription: Disables the power on the robot
		"""
		out_msg = self.send_command('hp 0')
		return out_msg

	def attach_robot(self, robot_id:str = "1", wait:int = 0.1):
		"""
		Decription: If there are multiple PF400 robots, chooses which robot will be programed attaches to the software. 
					If robot ID is not given it will attach the first robot.
		Parameters: 
				- robot_id: ID number of the robot
		"""
		out_msg = self.send_command("attach " + robot_id)
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
			for key, value in self.motion_profiles[0].items():
				cmd += ' ' + str(value)
			cmd2 = 'Profile 2'
			for key, value in self.motion_profiles[1].items():
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

		self.check_overall_state()

		if self.power_state == "-1":
			self.power_state = self.enable_power()
			sleep(6)

		if self.attach_state == "-1":
			self.attach_state = self.attach_robot()
			sleep(6) 
		
		if self.home_state == "-1":
			self.home_robot()
			sleep(6)

		profile = self.set_profile()

		if self.power_state[0].find("-") == -1 and self.attach_state[0].find("-") == -1 and profile[0].find("-") == -1:
			print("Robot initialization successfull")
		else:
			print("Robot initialization failed")

	def force_initialize_robot(self):
		"""
		Decription: Repeats the initilzation until there are no errors and the robot is initilzed.
		"""
		# Check robot state & initilize
		if self.check_overall_state() == -1:
			print("Robot is not intilized! Intilizing now...")
			self.initialize_robot()
			self.force_initialize_robot()

	def check_overall_state(self):
			"""
			Decription: Checks general state
			"""

			power_msg = self.send_command("hp").split(" ")
			# power_msg = power_msg.split(" ")

			attach_msg = self.send_command("attach").split(" ")
			# attach_msg = attach_msg.split(" ")

			home_msg = self.send_command("pd 2800").split(" ")
			# home_msg = home_msg.split(" ")

			state_msg = self.send_command("sysState").split(" ")
			# state_msg = state_msg.split(" ")

			if len(power_msg) == 1 or power_msg[0].find("-") != -1 or power_msg[1] == "0":
				self.power_state = "-1"
			else: 
				self.power_state = power_msg[1]

			if attach_msg[1].find("0") != -1 or attach_msg[0].find("-") != -1 or attach_msg[1] == "0":
				self.attach_state = "-1"
			else:
				self.attach_state = attach_msg[1]

			if home_msg[1].find("0") != -1 or home_msg[0].find("-") != -1 or home_msg[1] == "0":
				self.home_state = "-1"
			else: 
				self.home_state = home_msg[1]

			if state_msg[1].find("7") != -1 or state_msg[0].find("-") != -1:
				self.initialization_state = "-1"
			else: 
				self.initialization_state = state_msg[1]

			print("Power: " + self.power_state + " Attach: " + self.attach_state + " Home: " + self.home_state + " Robot State: " + self.initialization_state)

			if self.power_state == "-1" or self.attach_state == "-1" or self.home_state == "-1" or self.initialization_state == "-1":
				return -1
			else: 
				return 0

	## Get Commands 
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
		self.joint_state.raw_position = joint_array
		self.joint_state.position = [state * multiplier for state, multiplier in zip(joint_array, multipliers)]


	def find_cartesian_coordinates(self):
		"""
        Description: This function finds the current cartesian coordinates and angles of the robot.
		Return: A float array with x/y/z yaw/pich/roll
        """
		coordinates = self.send_command("whereC")
		coordinates_list = coordinates.split(' ')
		coordinates_list = coordinates_list[1:-1]
		return [float(x) for x in coordinates_list]

	def find_gripper_state(self):
		"""
		"""
		joints = self.find_joint_states()
		if float(joints[4]) > self.gripper_closed_state + 1.0:
			self.gripper_state = "open"
		else:
			self.gripper_state = 'closed'
		return self.gripper_state

	def forward_kinematics(self, joint_states):
		"""
		Desciption: Calculates the forward kinematics for a given array of joint_states. 
		Paramiters:
			- joint_states : 6 joint states of the target location/
		Return:
			- cartesian_coordinates: Returns the calculated cartesian coordinates of the given joint states
		"""
		cartesian_coordinates = self.find_cartesian_coordinates()
		shoulder_lenght = 225.0
		elbow_lenght = 210.0

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

	## Create Move commands
	def create_move_cartesian_command(self, target_cartesian_coordinates, profile:int =2):

		move_command = "MoveC"+ " " + str(profile) + " " + " ".join(map(str, target_cartesian_coordinates))
		return move_command

	def create_move_joint_command(self, target_joint_locations, profile:int = 1, gripper_close: bool = False, gripper_open: bool = False):
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
		## add check gripper here and remove gripper open/close from state
		if gripper_close == True and gripper_open == True:
			raise Exception("Gripper cannot be open and close at the same time!")
			
		# Setting the gripper location to open or close. If there is no gripper position passed in, target_joint_locations will be used.
		if gripper_close == True:
			target_joint_locations[4] = self.gripper_closed_state
		if gripper_open == True:
			target_joint_locations[4] = self.gripper_open_state

		move_command = "movej" + " " + str(profile) + " " + " ".join(map(str, target_joint_locations))

		return move_command		

	##Move commands

	# def move_joints(self, target_joint_locations, profile:int = 1, gripper_close: bool = False, gripper_open: bool = False):
	# 	return self.send_command(self.create_move_joint_command(target_joint_locations,profile))

	def move_in_one_axis_from_target(self, target_location, profile:int = 1, axis_x:int= 0,axis_y:int= 0, axis_z:int= 0):
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

		pass

	def move_in_one_axis(self,profile:int = 1, axis_x:int= 0,axis_y:int= 0, axis_z:int= 0):
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

	## lower order commands

	def gripper_open(self):
		""" Opens the gripper
		"""
		joint_locations = self.find_joint_states()
		joint_locations[4] = self.gripper_open_state
		self.send_command(self.create_move_joint_command(joint_locations,2))

		return self.find_gripper_state()

	def gripper_close(self):
		""" Closes the gripper
		"""
		joint_locations = self.find_joint_states()
		joint_locations[4] = self.gripper_closed_state
		self.send_command(self.create_move_joint_command(joint_locations,2))

		return self.find_gripper_state()

	def move_one_axis_with_rail(self, axis_num, target,pofile):
		""" Moves single axis to a target including the linear rail"""
		self.send_command("moveoneaxis " + str(axis_num) + str(target) + str(profile)) 

	def move_multiple_axis_with_rail(self, target1, target2):
		""" Moves extra two axises to their targets, including the linear rail"""
		self.send_command("moveextraaxis " + str(target1) + str(target2)) 
		pass

	def move_gripper_safe_zone(self):
		"""
		Description: Check if end effector is inside a module. If it is, move it on the y axis first to prevent collisions with the module frames.
		"""

		current_cartesian_coordinates = self.find_cartesian_coordinates()

		if current_cartesian_coordinates[1] <= self.module_left_dist:
			y_distance = self.module_left_dist - current_cartesian_coordinates[1] 
			self.move_in_one_axis(1,0,y_distance,0)
		elif current_cartesian_coordinates[1] >= self.module_right_dist:
			y_distance = self.module_right_dist - current_cartesian_coordinates[1]
			self.move_in_one_axis(1,0,y_distance,0)

	def move_gripper_neutral(self):
		"""
        Description: Move end effector to neutral position
        """
		
		# Create a new function to move the gripper into safe zone 
		self.move_gripper_safe_zone()
		gripper_neutral = self.find_joint_states()
		gripper_neutral[3] = 536.757

		self.send_command(self.create_move_joint_command(gripper_neutral,1))


	def move_arm_neutral(self):
		"""
        Description: Move arm to neutral position
        """
		arm_neutral = self.neutral_joints
		current_location = self.find_joint_states()
		arm_neutral[0] = current_location[0]
		arm_neutral[5] = current_location[5]
	

		self.send_command(self.create_move_joint_command(arm_neutral, 1))

	def move_rails_neutral(self, v_rail:float = None, h_rail:float = None):
		# Setting the target location's linear rail position for pf400_neutral 
		
		current_location = self.find_joint_states()

		if not v_rail:
			v_rail = current_location[0] # Keep the horizontal rail same
		if not h_rail:
			h_rail = current_location[5] # Keep the horizontal rail same

		self.neutral_joints[0] = v_rail + 60.0
		self.neutral_joints[5] = h_rail

		self.send_command(self.create_move_joint_command(self.neutral_joints))

	def move_all_joints_neutral(self, target_location):
		"""
        Description: Move all joints to neutral position
        """
		# First move end effector to it's nuetral position
		self.move_gripper_neutral()
		# Setting an arm neutral position without moving the horizontal & vertical rails
		self.move_arm_neutral()
		# Setting the target location's linear rail position for pf400_neutral 
		self.move_rails_neutral(target_location[0],target_location[5])


	def pick_plate(self, target_location):
		"""
        Description: 
        """
		slow_profile = 1
		fast_profile = 2

		abovePos = list(map(add, target_location, self.above))

		self.move_all_joints_neutral(target_location)
		self.send_command(self.create_move_joint_command(abovePos, fast_profile, False, True))
		self.send_command(self.create_move_joint_command(target_location, fast_profile, False, True))
		self.gripper_close()
		self.move_in_one_axis(profile = 1, axis_x = 0, axis_y = 0, axis_z = 60)
		self.move_all_joints_neutral(target_location)

		# TODO: USE BELOW MOVE_ONE_AXIS FUNCTIONS TO MOVE ABOVE AND FRONT OF THE EACH TARGET LOCATIONS
		# self.move_in_one_axis_from_target(target_location, profile = 2, axis_x = 60, axis_y = 0, axis_z = 60)
		# self.move_in_one_axis_from_target(target_location, profile = 1, axis_x = 0, axis_y = 0, axis_z = 60)

	def place_plate(self, target_location):
		"""
        Description: 
        """
		slow_profile = 1
		fast_profile = 2

		abovePos = list(map(add, target_location, self.above))

		self.move_all_joints_neutral(target_location)
		self.send_command(self.create_move_joint_command(abovePos, slow_profile, True, False))
		self.send_command(self.create_move_joint_command(target_location, slow_profile, True, False))
		self.gripper_open()
		self.move_in_one_axis(profile = 1, axis_x = 0, axis_y = 0, axis_z = 60)
		self.move_all_joints_neutral(target_location)


	def transfer(self, source, target):
		"""
        Description: Plate transfer function that performs series of movements to pick and place the plates
		
        """
		self.force_initialize_robot()
		self.pick_plate(source)
		self.place_plate(target)

if __name__ == "__main__":

	# from pf400_driver.pf400_driver import PF400
	robot = PF400("192.168.50.50", 10100)
	loc1 = [262.550, 20.608, 119.290, 662.570, 126.0, 574.367] #Hudson
	loc2 = [231.788, -27.154, 313.011, 342.317, 0.0, 683.702] #Sealer
	pos1= [262.550, 20.608, 119.290, 662.570, 0.0, 574.367] #Hudson
	pos2= [197.185, 59.736, 90.509, 566.953, 82.069, -65.550] #OT2
	thermocycler = [277.638, 39.029, 74.413, 602.159, 78.980, -910.338]

	# robot.transfer(pos1, pos2)
	# robot.transfer(pos2, thermocycler)
	# robot.transfer(thermocycler,pos1)

	# robot.send_command(robot.create_move_joint_command([322.544, 1.4, 177.101, 536.756, 78.933, 950]))
	# while int(robot.robot_movement_state()) != 1:
	# 	print(robot.robot_movement_state())
	# robot.move_all_joints_neutral([292, 20, 119, 662, 126, 574])


# TODO: Robot home skipped after enbamleing power took longer then the wait time. Fix wait times.
# TODO: Check if robot is homed by sending a dummy move command.
# TODO: HOME state command pd 2800

