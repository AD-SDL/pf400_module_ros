#!/usr/bin/env python3

import sys
import time

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

# # Begin example routine
client.SendCommand("wherej")
client.SendCommand("moveoneaxis 1 200 2")
client.SendCommand("moveoneaxis 2 -20 2")
client.SendCommand("moveoneaxis 3 300 2")
client.SendCommand("moveoneaxis 4 400 2")
client.SendCommand("moveoneaxis 1 300 2")
client.SendCommand("waitforeom")
client.SendCommand("attach 0")

client.Disconnect()
