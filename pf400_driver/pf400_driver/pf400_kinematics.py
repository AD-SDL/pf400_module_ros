
import math
import numpy as np
 

class KINEMATICS():
    def __init__(self):
        # Robot joint lenghts
        self.shoulder_lenght = 302
        self.elbow_lenght = 289
        self.end_effector_lenght = 162


    def forward_kinematics(self, joint_states:list):
        """
        Desciption: Calculates the forward kinematics for a given array of joint_states. 
        Paramiters:
            - joint_states : 6 joint states of the target location
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
        # print(yaw)    

        cartesian_coordinates = self.get_cartesian_coordinates()

        cartesian_coordinates[0] = round(x,3) + joint_states[5]
        cartesian_coordinates[1] = round(y,3)
        cartesian_coordinates[2] = round(z,3)
        cartesian_coordinates[3] = round(yaw,3)

        # print(round(x + joint_states[5], 3), round(y,3), round(z,3), round(phi,3))

        return cartesian_coordinates, round(phi,3), joint_states[5] 

    def inverse_kinematics(self, cartesian_coordinates:list, phi:float, rail:float = 0.0):

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

        if cartesian_coordinates[1] > 0 or (cartesian_coordinates[1] < 0 and math.degrees(theta1) < 0 and abs(math.degrees(theta1)) < abs(math.degrees(theta1 + 2 * gamma))):
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


