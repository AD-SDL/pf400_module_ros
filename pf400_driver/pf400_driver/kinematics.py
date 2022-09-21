
import math



def forward():
    cartesian_coordinates = [0,0,0,90,180,0]
    joint_states = [262.550, 20.608, 119.290, 662.570, 126.0, 574.367]
    shoulder_lenght = 225.0
    elbow_lenght = 210.0
    gripper_lengt = 0

    # Convert angles to radians
    shoulder_angle = joint_states[1]*math.pi/180 #Joint 2 
    elbow_angle = joint_states[2]*math.pi/180 #Joint 3
    gripper_angle = joint_states[3]*math.pi/180 # Joint 4

    # shoulder_angle = -15*math.pi/180 #Joint 2 
    # elbow_angle = 79*math.pi/180 #Joint 3
    # gripper_angle = -79*math.pi/180 # Joint 4

    x = shoulder_lenght*math.cos(shoulder_angle) + elbow_lenght*math.cos(shoulder_angle+elbow_angle) + gripper_lengt*math.cos(shoulder_angle+elbow_angle+gripper_angle) 
    y = shoulder_lenght*math.sin(shoulder_angle) + elbow_lenght*math.sin(shoulder_angle+elbow_angle)+ gripper_lengt*math.sin(shoulder_angle+elbow_angle+gripper_angle) 
    z = joint_states[0]

    cartesian_coordinates[0] = x
    cartesian_coordinates[1] = y
    cartesian_coordinates[2] = z
    phi = shoulder_angle + elbow_angle + gripper_angle
    print(x, y, z, phi)
    return(x,y,z, phi)


def inverse_kinematics(self, x, y, z, phi):

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
    

    print("theta1: {}".format(math.degrees(theta1), math.degrees(theta1 + 2 * gamma)))
    print("theta2: {} and {}".format(math.degrees(theta2), math.degrees(theta2 * - 1)))
    print("theta3: {} and {}".format(math.degrees(theta3), math.degrees(theta3 + 2 * (theta2 - gamma))))

    return [z, math.degrees(theta1), math.degrees(theta2), math.degrees(theta3)], [z, math.degrees(theta1 + 2 * gamma), math.degrees(theta2 * - 1), math.degrees(theta3 + 2 * (theta2 - gamma))]

x,y,z,phi = forward()
inverse_kinematics(x,y,z,phi)