
import math



def forward():
    # TODO: Seems like Joint 2 a weird range of motion. Find out Angle range and put an offset to the calculation
    # Joint to range 10-350
    # TODO: ADD X AXIS OFFSET AND LINEAR RAIL VALUE BEFORE RETURNING 

    cartesian_coordinates = [0,0,0,90,180,0]
    # joint_states = [231.788, -27.154, -124.011, 342.317, 0.0, 683.702] 
    # joint_states = [400.0, 1.400, 177.101, 536.757, 82.069, -65.550] 
    # joint_states =[231.788, -27.154, 313.011, 342.317, 0.0, 683.702]
    # joint_states = [264.584, -29.413, -100, 372.338, 0.0, 651.621]
    joint_states = [190.561, -58.794, 73.902, 1, 126.8, -446.223]

    if joint_states[2]>180:
        new_angle = -joint_states[2] + 180 # fixing the quadrant with angle range
    shoulder_lenght = 225.0
    elbow_lenght = 210.0
    gripper_lengt = 0.0

    # Convert angles to radians
    shoulder_angle = math.radians(joint_states[1]) #Joint 2 
    elbow_angle = math.radians(joint_states[2])#Joint 3
    gripper_angle = math.radians(joint_states[3]) # Joint 4

    # shoulder_angle = -15*math.pi/180 #Joint 2 
    # elbow_angle = 79*math.pi/180 #Joint 3
    # gripper_angle = -79*math.pi/180 # Joint 4

    x = shoulder_lenght*math.cos(shoulder_angle) + elbow_lenght*math.cos(shoulder_angle+elbow_angle) + gripper_lengt*math.cos(shoulder_angle+elbow_angle+gripper_angle) 
    y = shoulder_lenght*math.sin(shoulder_angle) + elbow_lenght*math.sin(shoulder_angle+elbow_angle)+ gripper_lengt*math.sin(shoulder_angle+elbow_angle+gripper_angle) 
    z = joint_states[0]

    cartesian_coordinates[0] = x
    cartesian_coordinates[1] = y
    cartesian_coordinates[2] = z
    phi = math.degrees(shoulder_angle) + math.degrees(elbow_angle) + math.degrees(gripper_angle)
    print(x, y, z, phi)
    return(x,y,z, phi)

def inverse(x,y,z,phi):

    # TODO: REMOVE X AXIS OFFSET AND LINEAR RAIL VALUE BEFORE CALCULATING 

    shoulder_lenght = 225.0
    elbow_lenght = 210.0
    gripper_lengt = 0.0

    theta2 = math.acos((x**2+y**2 - shoulder_lenght*shoulder_lenght - elbow_lenght*elbow_lenght)/(2*shoulder_lenght*elbow_lenght))
    
    if x < 0 and y < 0:
        theta2 = (-1) * theta2

    theta1 = math.atan(x / y) - math.atan((elbow_lenght * math.sin(theta2)) / (shoulder_lenght + elbow_lenght * math.cos(theta2)))
    
    theta2 = (-1) * theta2 * 180 / math.pi
    theta1 = theta1 * 180 / math.pi

    # Angles adjustment depending in which quadrant the final tool coordinate x,y is
    if x >= 0 and y >= 0:      # 1st quadrant
        theta1 = 90 - theta1


    if x < 0 and y > 0: 
          # 2nd quadrant
        theta1 = 90 - theta1


    if x < 0 and y < 0: 
          # 3d quadrant
        theta1 = 270 - theta1
        phi = 270 - theta1 - theta2
        phi = -1 * phi


    if x > 0 and y < 0: 
          # 4th quadrant
        theta1 = -90 - theta1


    if x < 0 and y == 0: 
            theta1 = 270 + theta1

    
    # Calculate "phi" angle so gripper is parallel to the X axis
    phi = 90 + theta1 + theta2
    phi = -1 * phi

    # Angle adjustment depending in which quadrant the final tool coordinate x,y is
    if x < 0 and y < 0: 
        # 3d quadrant
        phi = 270 - theta1 - theta2

    if abs(phi) > 165: 
        phi = 180 + phi

    theta1=round(theta1)
    theta2=round(theta2)
    phi=round(phi)
    print(theta1,theta2,phi)

def inverse_kinematics(x, y, z, phi):

    xe = x
    ye = y
    phie = math.radians(phi)

    shoulder_lenght = 225.0
    elbow_lenght = 210.0
    gripper_lengt = 0.0

    x_second_joint = xe - gripper_lengt * math.cos(phie) 
    y_second_joint = ye - gripper_lengt * math.sin(phie)

    radius = math.sqrt(x_second_joint**2 + y_second_joint**2) 
    gamma = math.acos((radius*radius + shoulder_lenght*shoulder_lenght - elbow_lenght*elbow_lenght)/(2*radius*shoulder_lenght)) 
    
    theta2 = math.pi - math.acos((shoulder_lenght*shoulder_lenght + elbow_lenght*elbow_lenght - radius*radius)/(2*shoulder_lenght*elbow_lenght))
    theta1 = math.atan2(y_second_joint, x_second_joint) - gamma 
    theta3 = phie - theta1 - theta2
    

    print("theta1: {} and {}".format(math.degrees(theta1), math.degrees(theta1 + 2 * gamma)))
    print("theta2: {} and {}".format(math.degrees(theta2), math.degrees(theta2 * - 1)))
    print("theta3: {} and {}".format(math.degrees(theta3), math.degrees(theta3 + 2 * (theta2 - gamma))))

    return [z, math.degrees(theta1), math.degrees(theta2), math.degrees(theta3)], [z, math.degrees(theta1 + 2 * gamma), math.degrees(theta2 * - 1), math.degrees(theta3 + 2 * (theta2 - gamma))]

x,y,z,phi = forward()
inverse_kinematics(x,y,z,phi)