#!/usr/bin/env python3

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

module_neutral = client.SendCommand("movej 1 300 0 237.92 388.47 77 643.54")

client.SendCommand("movej 2 300 -83.8 346.52 457.31 77 611.22")
client.SendCommand("movej 2 274.54 -83.8 346.52 457.31 90 611.22")
client.SendCommand("movej 2 300 -83.8 346.52 457.31 90 611.22")
client.SendCommand("movej 2 274.54 -83.8 346.52 457.31 90 611.22")
client.SendCommand("movej 2 274.54 -83.8 346.52 457.31 77 611.22")
client.SendCommand("movej 2 300 -83.8 346.52 457.31 77 611.22")


module_neutral = client.SendCommand("movej 1 300 0 237.92 388.47 77 643.54")

client.SendCommand("movej 2 240 -30.022 315.652 342.983 77 650.720")
client.SendCommand("movej 2 217 -30.022 315.652 342.983 77 650.720")
client.SendCommand("movej 2 217 -30.022 315.652 342.983 90 650.720")
client.SendCommand("movej 2 240 -30.022 315.652 342.983 90 650.720")
print("end of step 1")
client.SendCommand("movej 2 217 -30.022 315.652 342.983 90 650.720")
client.SendCommand("movej 2 217 -30.022 315.652 342.983 77 650.720")
print("end of step 2")
client.SendCommand("movej 2 240 -30.022 315.652 342.983 77 650.720")

module_neutral = client.SendCommand("movej 1 300 0 237.92 388.47 77 643.54")


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

client.SendCommand("wherej")
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
