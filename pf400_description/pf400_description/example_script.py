#!/usr/bin/env python3

from operator import mod
import sys
import time
from time import sleep
from TCSJointClient import TCSJointClient

client = TCSJointClient("192.168.50.50", "10100")

print("Beginning example routine")

# Enable high power if necessary
is_hp = client.SendCommand("hp")
if is_hp == "0 0":
	client.SendCommand("hp 1")
	time.sleep(5)

# Attach the robot to this thread
client.SendCommand("attach 1")

# # Home if necessary
is_homed = client.SendCommand("pd 2800")
if is_homed == "0 0": 
	client.SendCommand("home")


def open():
	client.SendCommand("attach 1")


# def close():
# 	pass

# de grab_sample():
# 	pass


# peller_top = client.SendCommand("movej 1 350 -86.67 345.2 460 95 643") 	 # move one axis
# peller_sample = client.SendCommand("movej 1 276.36 -86 347 460 95 643") 	 # move one axis
# peller_sample = client.SendCommand("movej 1 276.36 -86 347 460 79 643") 	 # move one axis
# peller_sample = client.SendCommand("movej 1 350 -86 347 460 79 643") 	 # move one axis
# ot2 = client.SendCommand("movej 1 314.86 88.48 279.07 442.24 79 650.728")
# module_neutral = client.SendCommand("movej 1 300 0 237.92 388.47 75.7 643.54")
# peller_top = client.SendCommand("movej 1 300 -34.15 322.98 339.06 75.7 650.728")
# peller_down_close = client.SendCommand("movej 1 212 -30.8 318.3 340.26 75.7 650.728")
# peller_down_open = client.SendCommand("movej 1 212 -30.8 318.3 340.26 100 650.728")

def moveJ(pos, profile=2):

	# if gripper=='Open':
	# 	pos[4]=plate_size
	# elif gripper=='Close':
	# 	pos[4]=90

	cmd = "movej" + " " + str(profile) + " " + " ".join(map(str, pos))

	print(cmd)
	client.SendCommand(cmd)

# def grab(joint):

	#check_gripper_pos

	#neutral
	#module_entry
	#pos_top
	#pos
	#close (can be just a moveaxis)
	#pos_top (can be just a moveaxis)
	#module_entry

# def drop():
# 	pass

# def transfer(pos1, pos2):
# 	pass




module_neutral = [400, 0, 237.92, 388.47, 76.7, 643.54]

peeler_above_open = [307.419, -31.460, 289.067, 373.143, 90.0, 661.437]
peeler_above_closed = [307.419, -31.460, 289.067, 373.143, 76.7, 661.437]

peeler_open = [272.419, -31.460, 289.067, 373.143, 90.0, 661.437]
peeler_closed = [272.419, -31.460, 289.067, 373.143, 76.7, 661.437]

sealer_above_closed = [250.966, -32.123, 304.570, 356.268, 76.7, 812.485]
sealer_above_open = [250.966, -32.123, 304.570, 356.268, 90.0, 812.485]

sealer_closed = [220.960, -30.670, 302.227, 356.440, 76.7, 812.442]
sealer_open = [220.960, -30.670, 302.227, 356.440, 90.0, 812.442]

module_switch = [260.241, -10.147, 174.616, 539.662, 76.7, -254.6]
ot2_enter = [260.241, -10.465, 60.89, 537.196, 76.7, -254.578]

ot2_above_closed = [231.183, 29.345, 89.351, 598.682, 76.7, -67.409]
ot2_above_open = [231.183, 29.345, 89.351, 598.682, 90.0, -67.409]

ot2_closed = [201.183, 29.345, 89.351, 598.682, 76.7, -67.409]
ot2_open = [201.183, 29.345, 89.351, 598.682, 90.0, -67.409]


# moveJ(module_neutral)

# moveJ(peeler_above_open)
# moveJ(peeler_open)
# sleep(2)
# moveJ(peeler_closed)
# moveJ(peeler_above_closed)
# moveJ(module_neutral)

# moveJ(sealer_above_closed)
# moveJ(sealer_closed)
# moveJ(sealer_open)
# moveJ(sealer_above_open)
# sleep(1)
# moveJ(module_neutral)


# moveJ(sealer_above_open)
# moveJ(sealer_open)
# sleep(2)
# moveJ(sealer_closed)
# moveJ(sealer_above_closed)
# moveJ(module_neutral)

# moveJ(module_switch)
# moveJ(ot2_enter)
# moveJ(ot2_above_closed)
# sleep(1)
# moveJ(ot2_closed)
# sleep(2)
# moveJ(ot2_open)
# sleep(1)
# moveJ(ot2_above_open)
# moveJ(ot2_enter)
# moveJ(module_switch)
# client.SendCommand("movej 2 300 -83.8 346.52 457.31 77 611.22")
# client.SendCommand("movej 2 274.54 -83.8 346.52 457.31 90 611.22")
# client.SendCommand("movej 2 300 -83.8 346.52 457.31 90 611.22")
# client.SendCommand("movej 2 274.54 -83.8 346.52 457.31 90 611.22")
# client.SendCommand("movej 2 274.54 -83.8 346.52 457.31 77 611.22")
# client.SendCommand("movej 2 300 -83.8 346.52 457.31 77 611.22")


moveJ(module_neutral)

# moveJ(sealer_above_open)
# moveJ(sealer_open)
# sleep(1)
# moveJ(sealer_closed)
# moveJ(sealer_above_closed)
# moveJ(module_neutral)


# moveJ(peeler_above_closed)
# moveJ(peeler_closed)
# sleep(2)
# moveJ(peeler_open)
# moveJ(peeler_above_open)
# moveJ(module_neutral)


moveJ(peeler_above_open)
moveJ(peeler_open)
sleep(1)
moveJ(peeler_closed)
moveJ(peeler_above_closed)
moveJ(module_neutral)

moveJ(module_switch)
moveJ(ot2_enter)
moveJ(ot2_above_closed)
sleep(1)
moveJ(ot2_closed)
sleep(2)
moveJ(ot2_open)
sleep(1)
moveJ(ot2_above_open)
moveJ(ot2_enter)
moveJ(module_switch)


# module_neutral = client.SendCommand("movej 1 300 0 237.92 388.47 77 643.54")

# client.SendCommand("movej 2 240 -30.022 315.652 342.983 77 650.720")
# client.SendCommand("movej 2 217 -30.022 315.652 342.983 77 650.720")
# client.SendCommand("movej 2 217 -30.022 315.652 342.983 90 650.720")
# client.SendCommand("movej 2 240 -30.022 315.652 342.983 90 650.720")
# print("end of step 1")
# client.SendCommand("movej 2 217 -30.022 315.652 342.983 90 650.720")
# client.SendCommand("movej 2 217 -30.022 315.652 342.983 77 650.720")
# print("end of step 2")
# client.SendCommand("movej 2 240 -30.022 315.652 342.983 77 650.720")

# module_neutral = client.SendCommand("movej 1 300 0 237.92 388.47 77 643.54")


####################################################
# Travel
# client.SendCommand("moveoneaxis 1 276.36 2") 	 # move one axis
# client.SendCommand("moveoneaxis 2 0 2")
# client.SendCommand("moveoneaxis 6 643.5 2")


#####################################################
#Before Sealer
# client.SendCommand("moveoneaxis 2 -40.67 2")
# client.SendCommand("moveoneaxis 3 250.2 2")
# client.SendCommand("moveoneaxis 3 345.2 2")
# client.SendCommand("moveoneaxis 2 -86.67 2")



#########################################
# Above Sealer

# client.SendCommand("wherej")
# client.SendCommand("moveoneaxis 1 276.36 2") 	 # move one axis
# client.SendCommand("moveoneaxis 2 -86.67 2")


# client.SendCommand("moveoneaxis 3 347.2 2")
# client.SendCommand("moveoneaxis 4 460.155 2")

###########################################
# Release Sample
#client.SendCommand("moveoneaxis 5 90 2")

# Return to place 


# Allow peeler to process sample


# client.SendCommand("waitforeom")
# client.SendCommand("attach 0")

client.Disconnect()
