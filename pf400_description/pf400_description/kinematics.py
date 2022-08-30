import math

shoulder_lenght = 225 
elbow_lenght = 210
joint_states = [1100, 5.9, 346]

# Convert angles to radians
shoulder_angle = joint_states[1]*math.pi/180 #Joint 2 
elbow_angle = joint_states[2]*math.pi/180 #Joint 3

x = shoulder_lenght*math.cos(shoulder_angle) + elbow_lenght*math.cos(shoulder_angle+elbow_angle) +175 + 683
y = shoulder_lenght*math.sin(shoulder_angle) + elbow_lenght*math.sin(shoulder_angle+elbow_angle)
print(x,y)