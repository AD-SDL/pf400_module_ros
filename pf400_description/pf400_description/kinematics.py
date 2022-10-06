
import math
import numpy as np
 



def forward():
    # TODO: Seems like Joint 2 has a weird range of motion. 
    # Joint two range 10-350

    cartesian_coordinates = [0,0,0,90,180,0]
    # joint_states = [231.788, -27.154, 124.011, 342.317, 0.0, 683.702] 
    joint_states = [400.0, 1.400, 177.101, 536.757, 82.069, -65.550] 
    # joint_states =[231.788, -27.154, 313.011, 342.317, 0.0, 683.702]
    # joint_states = [264.584, -29.413, -100, 372.338, 0.0, 651.621]
    # joint_states = [190.561, -58.794, 73.902, 1, 126.8, -446.223]
    # joint_states = [320.921, -27.747, 135.303, 699.950, 70.258, -0.102]
    # joint_states = [289.551, -21.918, 153.929, 526.774,	74.619, 574.345]
    # joint_states = [277.638, 39.029, 74.413, 602.149, 78.980, -910.338]
    # joint_states = [289.546, -21.973, 198.569, 555.221, 555.221, 574.337]


    if joint_states[2] > 180:
        adjusted_angle_j3 =  joint_states[2] - 360  # fixing the quadrant with angle range
    else:
        adjusted_angle_j3 = joint_states[2]

    shoulder_lenght = 302
    elbow_lenght = 289
    gripper_lengt = 162

    # Convert angles to radians
    shoulder_angle = math.radians(joint_states[1]) # Joint 2 
    elbow_angle = math.radians(joint_states[2]) # Joint 3
    gripper_angle = math.radians(joint_states[3]) # Joint 4

    # shoulder_angle = -15*math.pi/180 #Joint 2 
    # elbow_angle = 79*math.pi/180 #Joint 3
    # gripper_angle = -79*math.pi/180 # Joint 4

    x = shoulder_lenght*math.cos(shoulder_angle) + elbow_lenght*math.cos(shoulder_angle+elbow_angle) + gripper_lengt*math.cos(shoulder_angle+elbow_angle+gripper_angle) 
    y = shoulder_lenght*math.sin(shoulder_angle) + elbow_lenght*math.sin(shoulder_angle+elbow_angle) + gripper_lengt*math.sin(shoulder_angle+elbow_angle+gripper_angle) 
    z = joint_states[0]
    # yaw = math.degrees(math.atan(x/-y))
    # pitch = math.asin(y / 3)
    # yaw = math.asin( x / (math.cos(pitch)*3))  # //Beware cos(pitch)==0, catch this exception!
    # # roll = 0;    
    cartesian_coordinates[0] = x
    cartesian_coordinates[1] = y
    cartesian_coordinates[2] = z
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

    print(round(x + joint_states[5], 3), round(y,3), round(z,3), round(phi,3))

    return round(x,3), round(y,3), round(z,3), yaw,round(phi,3), joint_states[5]

    # TODO: ADD A YPR TO OPG CONVERTER FUNCTIONS`

def inverse_kinematics(x, y, z, yaw, phi):

    Joint_1 = z
    xe = x
    ye = y

    if phi < 360:
        phi = yaw
    elif phi > 360 and phi < 540:
        phi = yaw + 360
    elif phi > 540 and phi< 720:
        phi = yaw + 720 
    elif phi > 720 and phi < 900:
        phi = yaw + 720 
    elif phi > 900 and phi < 1080:
        phi = yaw + 1440

    phie = math.radians(phi)

    shoulder_lenght = 302
    elbow_lenght = 289
    gripper_lengt = 162

    x_second_joint = xe - gripper_lengt * math.cos(phie) 
    y_second_joint = ye - gripper_lengt * math.sin(phie)

    radius = math.sqrt(x_second_joint**2 + y_second_joint**2) 
    gamma = math.acos((radius*radius + shoulder_lenght*shoulder_lenght - elbow_lenght*elbow_lenght)/(2*radius*shoulder_lenght)) 

    theta2 = math.pi - math.acos((shoulder_lenght*shoulder_lenght + elbow_lenght*elbow_lenght - radius*radius)/(2*shoulder_lenght*elbow_lenght))
    theta1 = math.atan2(y_second_joint, x_second_joint) - gamma 
    theta3 = phie - theta1 - theta2

    if y > 0 or (y < 0 and math.degrees(theta1) < 0 and abs(math.degrees(theta1)) < abs(math.degrees(theta1 + 2 * gamma))):

        # Robot is in the First Quadrant on the coordinate plane (x:+ , y:+)
        Joint_2 = round(math.degrees(theta1),3)
        Joint_3 = round(math.degrees(theta2),3) # Adding 360 degrees to Joint 3 to fix the pose. 
        Joint_4 = round( math.degrees(theta3),3)
        # print("theta1: ", Joint_2)
        # print("theta2: ", Joint_3) 
        # print("theta3: ", Joint_4)

    elif y < 0:
        # Robot is in the Forth Quadrant on the coordinate plane (x:+ , y:-)
        # Use the joint angles for Forth Quadrant
        Joint_2 = round(math.degrees(theta1 + 2 * gamma),3)
        Joint_3 = round(math.degrees(theta2 * - 1) + 360, 3)# Adding 360 degrees to Joint 3 to fix the pose. 
        Joint_4 = round(math.degrees(theta3 + 2 * (theta2 - gamma)),3)
        # print("theta1: ", Joint_2)
        # print("theta2: ", Joint_3) 
        # print("theta3: ", Joint_4)

    return Joint_1, Joint_2, Joint_3, Joint_4

# x,y,z,yaw,phi,rail = forward()
# # print(x,y,z,phi,rail)

# # x,y,z,phi = 724.476, -62.664, 289.546,  #161.72699999999998, -180.428, 289.551, 720.76	
# Joint_1, Joint_2, Joint_3, Joint_4 = inverse_kinematics(x,y,z,yaw,phi)
# print(Joint_1, Joint_2, Joint_3, Joint_4)

#----
