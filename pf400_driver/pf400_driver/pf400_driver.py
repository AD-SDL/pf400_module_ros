#!/usr/bin/env python3

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
		self.power_state = ''
		self.home_state = ''
		self.gripper_state = ''

		self.connect()
		self.init_connection_mode()
		self.force_initialize_robot()

		##gripper vars
		self.gripper_open_length = 90.0
		self.gripper_closed_length = 79.0
		self.gripper_safe_height= 10.0
		self.gripper_state = self.find_gripper_state()

		##arm vars
		self.neutral_joints = [400.0, 0.0, 180.0, 530.993, self.gripper_closed_length, 0.0]	
		self.module_left_dist = -430.0
		self.module_right_dist = -430.0


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
			print(">> " + command)
			self.connection.write((command.encode("ascii") + b"\n"))
			response = self.connection.read_until(b"\r\n").rstrip().decode("ascii")
			if response != "" and response in self.error_codes:
				self.handle_error_output(response)
			else:
				print("<< "+ response)
			return response		
		finally:
			self.commandLock.release()

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
		"""
		"""
		if output in self.error_codes:
			print("<< " + self.error_codes[output])
		else:
			print("<< TCS Unknown error: " + output)

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

		self.power_state = self.send_command("hp")
		if self.power_state == "0 0":
			self.send_command("hp 1")
			sleep(5)

        # Attach the robot to this thread
		self.send_command("attach 1")

        # # Home if necessary
		self.home_state = self.send_command("pd 2800")
		if self.home_state == "0 0":
			self.send_command("home")

		# Set default motion profile		self.send_command("attach 1")

		self.set_profile()


	def force_initialize_robot(self):
		"""
		Decription: Repeats the initilzation until there are no errors and the robot is initilzed.
		"""
		# Check robot state & initilize
		if self.check_general_state() == -1:
			print("Robot is not intilized! Intilizing now...")
			self.initialize_robot()
			self.force_initialize_robot()

	def check_general_state(self, wait:int = 0.1):
			"""
			Decription: Checks general state
			"""

			power_msg = self.send_command("hp")
			power_msg = power_msg.split(" ")

			attach_msg = self.send_command("attach")
			attach_msg = attach_msg.split(" ")

			state_msg = self.send_command("sysState")
			state_msg = state_msg.split(" ")

			power ,attach, state = 0, 0, 0

			if len(power_msg) == 1 or power_msg[0].find("-") != -1:
				power = -1
			if attach_msg[1].find("0") != -1 or attach_msg[0].find("-") != -1:
				attach = -1
			if state_msg[1].find("7") != -1 or state_msg[0].find("-") != -1:
				state = -1
			print(power, attach, state)
			if power == -1 or attach == -1 or state == -1:
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
		if float(joints[4]) > self.gripper_closed_length + 1.0:
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
			target_joint_locations[4] = self.gripper_closed_length
		if gripper_open == True:
			target_joint_locations[4] = self.gripper_open_length

		move_command = "movej" + " " + str(profile) + " " + " ".join(map(str, target_joint_locations))

		return move_command		

	##Move commands

	# def move_joints(self, target_joint_locations, profile:int = 1, gripper_close: bool = False, gripper_open: bool = False):
	# 	return self.send_command(self.create_move_joint_command(target_joint_locations,profile))

	# def move_in_one_axis_from_target(self, target_location, profile:int = 2, axis_x:int= 0,axis_y:int= 0, axis_z:int= 0):
	# 	"""
	# 	TODO: TRY THIS FUNCTION TO SEE IF END EFFECTOR MOVES ON A SINGLE AXIS PROPERLY

	# 	Desciption: Moves the end effector on single axis with a goal movement in milimeters. 
	# 	Paramiters:
	# 		- target_location : Joint states of the target location
	# 		- axis_x : Goal movement on x axis in mm
	# 		- axis_y : Goal movement on y axis in mm
	# 		- axis_z : Goal movement on z axis in mm
	# 	"""
	# 	# First move robot on linear rail
	# 	current_joint_state = self.find_joint_states()
	# 	current_joint_state[5] = target_location[5]
	# 	self.send_command(self.create_move_joint_command(current_joint_state))

	# 	# Find the cartesian coordinates of the target joint states
	# 	cartesian_coordinates = self.forward_kinematics(target_location)
		
	# 	# Move en effector on the single axis
	# 	cartesian_coordinates[0] += axis_x
	# 	cartesian_coordinates[1] += axis_y
	# 	cartesian_coordinates[2] += axis_z

	# 	move_command = "MoveC "+ " " + str(profile) + " " + "".join(map(str, cartesian_coordinates))
	# 	self.send_command(move_command)

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

	## lower order commands
	def gripper_open(self):
		"""
		"""
		joint_locations = self.find_joint_states()
		print(joint_locations)
		joint_locations[4] = self.gripper_open_length
		self.send_command(self.create_move_joint_command(joint_locations,2))
		sleep(1)
		return self.find_gripper_state()

	def gripper_close(self):
		"""
		"""
		joint_locations = self.find_joint_states()
		joint_locations[4] = self.gripper_closed_length
		self.send_command(self.create_move_joint_command(joint_locations,2))
		sleep(1)
		return self.find_gripper_state()

	def set_gripper_neutral(self):
		"""
        """
		current_joint_locations = self.find_joint_states()
		current_cartesian_coordinates = self.find_cartesian_coordinates()
		if current_cartesian_coordinates[1] > 0:
			self.move_in_one_axis(1,0,-100,0)
		else:
			self.move_in_one_axis(1,0,100,0)
		
		current_joint_locations[3] = self.neutral_joints[3]
		self.send_command(self.create_move_joint_command(current_joint_locations,2))

	def move_end_effector_neutral(self):
		"""
        Description: Move end effector to neutral position
        """
		current_joint_locations = self.find_joint_states()
		
		current_cartesian_coordinates = self.find_cartesian_coordinates()
		
		# Check if end effector is inside a module. If it is, move it on the y axis first to prevent collisions with the module frames.
		safe_y_distance = -430
		if current_cartesian_coordinates[1] <= safe_y_distance:
			y_distance = safe_y_distance - current_cartesian_coordinates[1] 
			self.move_in_one_axis(1,0,y_distance,0)

		current_joint_locations[3] = 530.993

		self.send_command(self.create_move_joint_command(current_joint_locations, 2, True, False))


	def move_arm_neutral(self, height=None, rail=None):
		"""
        Description: Move all joints to neutral position
        """

		neutral = self.neutral_joints
		current_joint_locations = self.find_joint_states()
		print(current_joint_locations)
		print(neutral)

		if not height:
			height=current_joint_locations[0]
		if not rail:
			rail=current_joint_locations[5]

		neutral[0] = height
		neutral[5] = rail
	
		self.set_gripper_neutral()

		self.send_command(self.create_move_joint_command(neutral,2))

	def move_all_joints_neutral(self, target_location):
		"""
        Description: Move all joints to neutral position
        """
		# First move end effector to it's nuetral position
		self.move_end_effector_neutral()

		# Setting an arm neutral position without moving the linear rail
		current_location = self.find_joint_states()
		arm_neutral = self.neutral_joints
		arm_neutral[5] = current_location[5]
		self.send_command(self.create_move_joint_command(arm_neutral))

		# Setting the target location's linear rail position for pf400_neutral 
		self.neutral_joints[5] = target_location[5]
		self.send_command(self.create_move_joint_command(self.neutral_joints))

	def pick_plate(self, target_location):
		"""
        Description: 
        """
		slow_profile = 1
		fast_profile = 2

		abovePos = list(map(add, target_location, self.above))


		#raf has an extra command here
		self.move_arm_neutral()
		self.move_arm_neutral(rail=target_location[5],height=abovePos[0])
		# self.send_command(self.create_move_joint_command(entryPos, fast_profile, False, True))
		self.send_command(self.create_move_joint_command(abovePos, fast_profile, False, True))
		self.send_command(self.create_move_joint_command(target_location, slow_profile, False, True))
		# self.gripper_close()
		self.send_command(self.create_move_joint_command(target_location, slow_profile, True, True))
		self.send_command(self.create_move_joint_command(abovePos))
		# self.send_command(self.create_move_joint_command(entryPos, slow_profile, True, False))
		sleep(1)
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

		sleep(1)
		#self.move_all_joints_neutral(target_location)
		#raf has an extra command here
		self.move_arm_neutral()
		self.move_arm_neutral(rail=target_location[5],height=abovePos[0])
		self.send_command(self.create_move_joint_command(abovePos, fast_profile, True, False))
		self.send_command(self.create_move_joint_command(target_location, slow_profile, True, False))
 		#self.gripper_open()
		self.send_command(self.create_move_joint_command(target_location, slow_profile, False, True))
		self.send_command(self.create_move_joint_command(abovePos))
		sleep(1)
		self.move_arm_neutral(rail=target_location[5],height=abovePos[0])
		#self.move_all_joints_neutral(target_location)


	def transfer(self, source, dest):
		"""
        Description: Plate transfer function that performs series of movements to pick and place the plates
		
        """
		self.force_initialize_robot()
		self.pick_plate(source)
		self.place_plate(dest)

if __name__ == "__main__":

	from pf400_driver.pf400_driver import PF400
	robot = PF400("192.168.50.50", 10100)
	loc1 = [262.550, 20.608, 119.290, 662.570, 126.0, 574.367]
	loc2 = [231.788, -27.154, 313.011, 342.317, 0.0, 683.702]

	robot.transfer(loc1, loc2)
	robot.transfer(loc2, loc1)



