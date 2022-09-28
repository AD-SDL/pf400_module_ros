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

	def __init__(self, host= "192.168.50.50", port = 10100, mode = 0):
		
		"""
        Description: 
			- Python interface that allows remote commands to be executed using simple string messages over Telnet socket on PF400. 
			- PF400 is the main object that will be used for operations such as remote connection as well as sending movement commands.
			- Programs are sent to the 10x00 port (first robot port: 10100). 
			- A program sent to robot will be executed immediately unless there is a prior operation running on the robot. 
			- If a second motion command is sent while the referenced robot is moving, the second command is blocked and will not reply until the first motion is complete.

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

		# Robot State
		self.power_state = "0"
		self.attach_state = "0"
		self.home_state = "1"
		self.initialization_state = "0"
		self.movement_state = -2
		self.robot_state = "Normal"

		# Initialize robot 
		self.connect()
		self.init_connection_mode()
		self.force_initialize_robot()
		
		# Robot kinematics
		self.shoulder_lenght = 302
		self.elbow_lenght = 289
		self.end_effector_lenght = 162

		# Gripper variables
		self.gripper_open_state = 130.0
		self.gripper_closed_state = 77.0
		self.gripper_safe_height = 10.0
		self.set_gripper_open()
		self.set_gripper_close()
		self.gripper_state = self.get_gripper_state()

		# Arm variables
		self.neutral_joints = [400.0, 1.400, 177.101, 536.757, self.gripper_closed_state, 0.0]	
		self.module_left_dist = -420.0
		self.module_right_dist = 220.0

		# Sample variables
		self.sample_above_height = 60.0
		self.above = [self.sample_above_height,0,0,0,0,0]
		self.y_recoil = 300.0	

		# Plate variables
		self.plate_state = 0
		self.plate_width = 123
		self.plate_source_rotation = 0 # 90 to rotate 90 degrees
		self.plate_target_rotation = 0 # 90 to rotate 90 degrees
		self.plate_ratation_deck = [231.788, -27.154, 313.011, 342.317, 0.0, 683.702] # Set Peeler location

	def connect(self):
		"""
		Decription: Create a streaming socket to send string commands to the robot. 
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
		Decription: Sends the commands to the robot over the socket client
        Parameters: 
                - command: Command itself in string format
        """

		self.commandLock.acquire()
		
		try:
			if not self.connection:
				self.Connect()	

			self.get_robot_movement_state()
			if self.movement_state > 1:
				print("Waiting for robot movement to end before sending the new command")
				while self.movement_state > 1:
					self.get_robot_movement_state()

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

	# INITIALIZE COMMANDS 

	def init_connection_mode(self):
		"""
        """
		if not self.connection:
			self.Connect()
		if self.mode == 0:
			# Set TCS to nonverbose
			self.connection.write(("mode 0".encode("ascii") + b"\n"))
		else:
			# Set TCS to verbose
			self.connection.write(("mode 1".encode("ascii") + b"\n"))
			self.send_command("selectrobot 1")
			
		init_mode = self.connection.read_until(b"\r\n").rstrip().decode("ascii")

	def handle_error_output(self, output):
		"""
		Decription: Handles the error message output
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

		out_msg = self.send_command(cmd)
		sleep(10)

		return out_msg

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
		

			out_msg = self.send_command(cmd)
			out_msg2 = self.send_command(cmd2)

		elif len(profile_dict) == 8:


			cmd = 'Profile 3'
			for key, value in profile_dict.items():
				cmd += ' ' + str(value)

			out_msg = self.send_command(cmd)
			
		else:
			raise Exception("Motion profile takes 8 arguments, {} where given".format(len(profile_dict)))

		return out_msg 

	def initialize_robot(self):
		"""
		Decription: Intilizes the robot by calling enable_power, attach_robot, home_robot, set_profile functions and 
					checks the robot state to find out if the initilization was successful
		"""

		self.get_overall_state()

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
		# self.set_gripper_open()
		# self.set_gripper_close()

		if self.power_state[0].find("-") == -1 and self.attach_state[0].find("-") == -1 and profile[0].find("-") == -1:
			print("Robot initialization successfull")
		else:
			print("Robot initialization failed")

	def force_initialize_robot(self):
		"""
		Decription: Repeats the initilzation until there are no errors and the robot is initilzed.
		"""
		# Check robot state & initilize
		if self.get_overall_state() == -1:
			print("Robot is not intilized! Intilizing now...")
			self.initialize_robot()
			self.force_initialize_robot()

	# GET COMMANDS

	def get_robot_movement_state(self):
		"""Checks the movement state of the robot
		States: 0 = Power off
				1 = Stopping
				2 = Acceleration
				3 = Decelaration	
		"""

		self.connection.write(("state".encode("ascii") + b"\n"))
	
		movement_state = self.connection.read_until(b"\r\n").rstrip().decode("ascii")

		if movement_state != "" and movement_state in self.error_codes:
			self.handle_error_output(movement_state)
		else:
			self.movement_state = int(movement_state.split()[1])
			
	def get_overall_state(self):
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

	def get_joint_states(self):
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
		joint_array = self.get_joint_states()
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

	def get_cartesian_coordinates(self):
		"""
        Description: This function finds the current cartesian coordinates and angles of the robot.
		Return: A float array with x/y/z yaw/pich/roll
        """
		coordinates = self.send_command("whereC")
		coordinates_list = coordinates.split(' ')
		coordinates_list = coordinates_list[1:-1]
		return [float(x) for x in coordinates_list]

	def get_gripper_state(self):
		"""
		"""
		joints = self.get_joint_states()
		if float(joints[4]) > self.gripper_closed_state + 1.0:
			self.gripper_state = "open"
		else:
			self.gripper_state = 'closed'
		return self.gripper_state

	def get_gripper_lenght(self):
		joint_angles = self.get_joint_states()
		return joint_angles[4]

	# SET COMMANDS

	def set_gripper_open(self):
		self.send_command("GripOpenPos " + self.gripper_open)
		
	def set_gripper_close(self):
		self.send_command("GripClosePos " + self.gripper_close)

	def set_plate_rotation(self, joint_states, rotation_degree = 0):
		"""
		Description:
		Parameters:
			- joint_states:
			- rotation_degree: 
		"""
		cartesian_coordinates, phi_angle, rail_pos = self.forward_kinematics(joint_states)

		if cartesian_coordinates[1] < 0:
			#Location is on the right side of the robot
			cartesian_coordinates[3] += rotation_degree
		elif cartesian_coordinates[1] > 0:
			cartesian_coordinates[3] -= rotation_degree
		
		new_joint_angles = self.inverse_kinematics(cartesian_coordinates, phi_angle, rail_pos)

		return new_joint_angles


	# KINEMATIC CALCULATIONS

	def forward_kinematics(self, joint_states:list):
		"""
		Desciption: Calculates the forward kinematics for a given array of joint_states. 
		Paramiters:
			- joint_states : 6 joint states of the target location/
		Return:
			- cartesian_coordinates: Returns the calculated cartesian coordinates of the given joint states
			- phi: Phi angle in degress to be used for inverse kinematics
			- joint_state[5]: The rail lenght. Needs to be supstracted from x axis if calculated coordinates will be fed into inverse kinematics
		"""
		
		if joint_states[2] > 180:
			adjusted_angle_j3 =  joint_states[2] - 360  # Fixing the quadrant on the third joint. Joint 3 range is 10 to 350 instead of -180 to 180
		else:
			adjusted_angle_j3 = joint_states[2]

		# Convert angles to radians
		shoulder_angle = math.radians(joint_states[1]) # Joint 2 
		elbow_angle = math.radians(joint_states[2]) # Joint 3
		gripper_angle = math.radians(joint_states[3]) # Joint 4

		
		x = self.shoulder_lenght*math.cos(shoulder_angle) + self.elbow_lenght*math.cos(shoulder_angle+elbow_angle) + self.end_effector_lenght*math.cos(shoulder_angle+elbow_angle+gripper_angle) 
		y = self.shoulder_lenght*math.sin(shoulder_angle) + self.elbow_lenght*math.sin(shoulder_angle+elbow_angle) + self.end_effector_lenght*math.sin(shoulder_angle+elbow_angle+gripper_angle) 
		z = joint_states[0]
		
		phi = math.degrees(shoulder_angle) + adjusted_angle_j3 + math.degrees(gripper_angle)

		if phi > 0 and phi < 540:
			yaw = phi%360
		elif phi > 540 and phi<720:
			yaw = phi%360 - 360
		elif phi > 720 and phi < 900:
			yaw = phi%720
		elif phi > 900 and phi < 1080:
			yaw = phi%720 - 720
		print(yaw)    

		cartesian_coordinates = self.get_cartesian_coordinates()

		cartesian_coordinates[0] = round(x,3) + joint_states[5]
		cartesian_coordinates[1] = round(y,3)
		cartesian_coordinates[2] = round(z,3)
		cartesian_coordinates[3] = round(yaw,3)

		print(round(x + joint_states[5], 3), round(y,3), round(z,3), round(phi,3))

		return cartesian_coordinates, round(phi,3), joint_states[5] 

	def inverse_kinematics(self, cartesian_coordinates:list, phi:float, rail:float = 0):
		
		"""
		Desciption: Calculates the inverse kinematics for a given array of cartesian coordinates. 
		Paramiters:
			- cartesian_coordinates: X/Y/Z Yaw/Pitch/Roll cartesian coordinates. 
									 X axis has to be substracted from the rail length before feeding into this function!
			- Phi: Phi angle. Phi = Joint_2_angle + Joint_3_angle + Joint_4_angle
			- Rail: Rail length (optional). If provided it will be substracted from X axis.
		Return:
			- Joint angles: Calculated 6 new joint angles.
		"""
			
		Joint_1 = cartesian_coordinates[2]
		xe = cartesian_coordinates[0] - rail
		ye = cartesian_coordinates[1]

		if phi < 360:
			phi = cartesian_coordinates[3]
		elif phi > 360 and phi < 540:
			phi = cartesian_coordinates[3] + 360
		elif phi > 540 and phi< 720:
			phi = cartesian_coordinates[3] + 720 
		elif phi > 720 and phi < 900:
			phi = cartesian_coordinates[3] + 720 
		elif phi > 900 and phi < 1080:
			phi = cartesian_coordinates[3] + 1440

		phie = math.radians(phi)

		x_second_joint = xe - self.end_effector_lenght * math.cos(phie) 
		y_second_joint = ye - self.end_effector_lenght * math.sin(phie)

		radius = math.sqrt(x_second_joint**2 + y_second_joint**2) 
		gamma = math.acos((radius*radius + self.shoulder_lenght * self.shoulder_lenght - self.elbow_lenght * self.elbow_lenght)/(2 * radius * self.shoulder_lenght)) 
		
		theta2 = math.pi - math.acos((self.shoulder_lenght * self.shoulder_lenght + self.elbow_lenght * self.elbow_lenght - radius*radius)/(2 * self.shoulder_lenght * self.elbow_lenght))
		theta1 = math.atan2(y_second_joint, x_second_joint) - gamma 
		theta3 = phie - theta1 - theta2
		
		if [1] > 0 or (cartesian_coordinates[1] < 0 and math.degrees(theta1) < 0 and abs(math.degrees(theta1)) < abs(math.degrees(theta1 + 2 * gamma))):
			# Robot is in the First Quadrant on the coordinate plane (x:+ , y:+)
			Joint_2 = math.degrees(theta1)
			Joint_3 = math.degrees(theta2) # Adding 360 degrees to Joint 3 to fix the pose. 
			Joint_4 = math.degrees(theta3)
			# print("theta1: ", Joint_2)
			# print("theta2: ", Joint_3) 
			# print("theta3: ", Joint_4)

		elif cartesian_coordinates[1] < 0:
			# Robot is in the Forth Quadrant on the coordinate plane (x:+ , y:-)
			# Use the joint angles for Forth Quadrant
			Joint_2 = math.degrees(theta1 + 2 * gamma)
			Joint_3 = math.degrees(theta2 * - 1) + 360 # Adding 360 degrees to Joint 3 to fix the pose. 
			Joint_4 = math.degrees(theta3 + 2 * (theta2 - gamma))
			# print("theta1: ", Joint_2)
			# print("theta2: ", Joint_3) 
			# print("theta3: ", Joint_4)

		return [Joint_1, Joint_2, Joint_3, Joint_4, self.get_gripper_lenght(), rail]

	# MOVE COMMANDS
	def create_move_cartesian_command(self, target_cartesian_coordinates, profile:int =2):

		move_command = "MoveC"+ " " + str(profile) + " " + " ".join(map(str, target_cartesian_coordinates))
		return move_command

	def create_move_joint_command(self, target_joint_angles, profile:int = 1, gripper_close: bool = False, gripper_open: bool = False):
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
			
		# Setting the gripper location to open or close. If there is no gripper position passed in, target_joint_angles will be used.
		if gripper_close == True:
			target_joint_angles[4] = self.gripper_closed_state
		if gripper_open == True:
			target_joint_angles[4] = self.gripper_open_state

		move_command = "movej" + " " + str(profile) + " " + " ".join(map(str, target_joint_angles))

		return move_command		

	# def move_joints(self, target_joint_angles, profile:int = 1, gripper_close: bool = False, gripper_open: bool = False):
	# 	return self.send_command(self.create_move_joint_command(target_joint_angles,profile))

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
		current_joint_state = self.get_joint_states()
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
		cartesian_coordinates = self.get_cartesian_coordinates()
		
		# Move end effector on the single axis
		cartesian_coordinates[0] += axis_x
		cartesian_coordinates[1] += axis_y
		cartesian_coordinates[2] += axis_z

		move_command = "MoveC"+ " " + str(profile) + " " + " ".join(map(str, cartesian_coordinates))
		self.send_command(move_command)

	def grab_plate(self, width: int = 123, speed:int = 100, force: int = 10):
		""" 
		Description: 
			Grabs the plate by appling additional force
		Parameters:
			- width: Plate width, in mm. Should be accurate to within about 1 mm.
			- speed: Percent speed to open fingers.  1 to 100.
			- Force: Maximum gripper squeeze force, in Nt. 
					 A positive value indicates the fingers must close to grasp.  
					 A negative value indicates the fingers must open to grasp.
		Returns:
			- 1: Plate grabed
			- 0: Plate is not grabed
		"""
		grab_plate_status = self.send_command("GraspPlate " + str(width)+ " " + str(speed) + " " + str(force)).split(" ")
		
		if len(grab_plate_status) < 2:
			return

		elif grab_plate_status[1] == "-1":
			print("Plate is grabed")
			self.plate_state = 1
			self.gripper_closed_state = width

		elif grab_plate_status[1] == "0" and width > 80: # Do not try smaller width 
			print("No plate") 
			width -= 1
			self.grab_plate(width,speed,force)

		elif width <= 80:
			print("PLATE WAS NOT FOUND!")
			self.robot_state = "Missing Plate"
			# TODO: Stop robot transfer here
			self.plate_state = -1
		return grab_plate_status

	def release_plate(self, width: int = 130, speed:int = 100):
		""" 
		Description: 
			Release the plate 
		Parameters:
			- width: Open width, in mm.
			- speed: Percent speed to open fingers.  1 to 100.
		Returns:
			- Plate released
			- 
		"""

		release_plate_status = self.send_command("ReleasePlate " + str(width)+ " " + str(speed)).split(" ")

		if release_plate_status[0] == "1":
			print("Plate is not released")
		elif release_plate_status[0] == "0":
			print("Plate is released") 
			self.plate_state = 0

		return release_plate_status

	def gripper_open(self):
		""" Opens the gripper
		"""
		self.send_command("gripper 1")
		return self.get_gripper_state()

	def gripper_close(self):
		""" Closes the gripper
		"""
		self.send_command("gripper 2")
		return self.get_gripper_state()

	def move_one_axis_with_rail(self, axis_num, target,pofile):
		""" Moves single axis to a target"""
		self.send_command("moveoneaxis " + str(axis_num) + str(target) + str(profile)) 

	def move_multiple_axis_with_rail(self, target1, target2):
		""" Moves extra two axises to their targets"""
		self.send_command("moveextraaxis " + str(target1) + str(target2)) 
		pass

	def move_gripper_safe_zone(self):
		"""
		Description: Check if end effector is inside a module. If it is, move it on the y axis first to prevent collisions with the module frames.
		"""

		current_cartesian_coordinates = self.get_cartesian_coordinates()

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
		gripper_neutral = self.get_joint_states()
		gripper_neutral[3] = 536.757

		self.send_command(self.create_move_joint_command(gripper_neutral,1))


	def move_arm_neutral(self):
		"""
        Description: Move arm to neutral position
        """
		arm_neutral = self.neutral_joints
		current_location = self.get_joint_states()
		arm_neutral[0] = current_location[0]
		arm_neutral[5] = current_location[5]
	

		self.send_command(self.create_move_joint_command(arm_neutral, 1))

	def move_rails_neutral(self, v_rail:float = None, h_rail:float = None):
		# Setting the target location's linear rail position for pf400_neutral 
		
		current_location = self.get_joint_states()

		if not v_rail:
			v_rail = current_location[0] # Keep the horizontal rail same
		if not h_rail:
			h_rail = current_location[5] # Keep the horizontal rail same

		self.neutral_joints[0] = v_rail + 60.0
		self.neutral_joints[5] = h_rail

		self.send_command(self.create_move_joint_command(self.neutral_joints))

	def move_all_joints_neutral(self, target_location = None):
		"""
        Description: Move all joints to neutral position
        """
		if target_location == None:
			target_location = self.get_joint_states()
		# First move end effector to it's nuetral position
		self.move_gripper_neutral()
		# Setting an arm neutral position without moving the horizontal & vertical rails
		self.move_arm_neutral()
		# Setting the target location's linear rail position for pf400_neutral 
		self.move_rails_neutral(target_location[0],target_location[5])


	def pick_plate(self, source_location):
		"""
        Description: 
        """
		slow_profile = 1
		fast_profile = 2

		abovePos = list(map(add, source_location, self.above))

		self.move_all_joints_neutral(source_location)
		self.send_command(self.create_move_joint_command(abovePos, fast_profile, False, True))
		self.send_command(self.create_move_joint_command(source_location, fast_profile, False, True))
			
		self.grab_plate(self.plate_width,100,10)
		self.move_in_one_axis(profile = 1, axis_x = 0, axis_y = 0, axis_z = 60)
		self.move_all_joints_neutral(source_location)

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
		self.release_plate()
		self.move_in_one_axis(profile = 1, axis_x = 0, axis_y = 0, axis_z = 60)
		self.move_all_joints_neutral(target_location)

	def rotate_plate_on_deck(self, rotation_degree):
		"""Uses the deck to ratate the plate between two transfers"""
		target = self.plate_ratation_deck


		if rotation_degree < 0:
			target = self.set_plate_rotation(target, rotation_degree)

		abovePos = list(map(add, target, self.above))

		self.move_all_joints_neutral(target)
		self.send_command(self.create_move_joint_command(abovePos, 1, True, False))
		self.send_command(self.create_move_joint_command(target, 1, True, False))
		self.release_plate()
		self.move_in_one_axis(profile = 1, axis_x = 0, axis_y = 0, axis_z = 60)

		# Ratating gripper to grab the plate from other rotation
		if rotation_degree > 0 :	
			target = self.set_plate_rotation(target, rotation_degree)

		self.send_command(self.create_move_joint_command(target, 1, False, True))
		self.grab_plate(self.plate_width,100,10)
		self.move_in_one_axis(profile = 1, axis_x = 0, axis_y = 0, axis_z = 60)
		self.move_all_joints_neutral(target)

	def transfer(self, source:list, target:list, source_plate_rotation:int = 0, target_plate_rotation:int = 0):
		"""
        Description: Plate transfer function that performs series of movements to pick and place the plates
		Parameters: 
			- source: Source location
			- target: Targer location
			- source_plate_rotation: 0 or 90 degrees
			- target_plate_rotation: 0 or 90 degrees

		Note: Plate rotation defines the rotation of the plate on the deck, not the grabing angle.
		
        """

		self.plate_source_rotation = source_plate_rotation
		self.plate_target_rotation = target_plate_rotation

		# These two will fix plate rotation on the deck
		if self.plate_source_rotation != 0 and source[3] > 400 and source[3] < 550:
			source = self.set_plate_rotation(source, self.plate_source_rotation)
		
		if self.plate_target_rotation != 0 and target[3] > 400 and target[3] < 550: 
			target = self.set_plate_rotation(target, self.plate_target_rotation)


		self.force_initialize_robot()
		self.pick_plate(source)

		if self.plate_state == -1: 
			print("Transfer cannot be completed, missing plate!")
			return # Stopping transfer here

		if self.plate_source_rotation != 0 and self.plate_target_rotation == 0:
			# Need a transition from 90 degree to 0 degree
			self.rotate_plate_on_deck(-self.plate_source_rotation)

		if self.plate_source_rotation == 0 and self.plate_target_rotation != 0:
			self.rotate_plate_on_deck(self.plate_target_rotation)
			# Need a transition from 0 degree to 90 degree

		self.place_plate(target)

if __name__ == "__main__":

	# from pf400_driver.pf400_driver import PF400
	robot = PF400("192.168.50.50", 10100)
	loc1 = [262.550, 20.608, 119.290, 662.570, 126.0, 574.367] #Hudson
	loc2 = [231.788, -27.154, 313.011, 342.317, 0.0, 683.702] #Sealer
	pos1= [262.550, 20.608, 119.290, 662.570, 0.0, 574.367] #Hudson
	pos2= [197.185, 59.736, 90.509, 566.953, 82.069, -65.550] #OT2
	thermocycler = [277.638, 39.029, 74.413, 602.159, 78.980, -910.338]

	robot.transfer(pos1, pos2)
	robot.transfer(pos2,pos1)
	# robot.transfer(pos2, loc2)
	# robot.transfer(loc2,pos1)




