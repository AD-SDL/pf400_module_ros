import socket
import time
import logging

#Log Configuration
logging.basicConfig(filename = "logs/robot_client_logs.log", level=logging.DEBUG, format = '[%(levelname)s] [%(asctime)s] [%(name)s] %(message)s', datefmt = '%Y-%m-%d %H:%M:%S')

class PF400(object):
    """
    Description: 
                 - Python interface that allows remote commands to be executed using simple string messages over TCP/IP on PF400 cobot. 
                 - Arm_Robot is the main object that will be used for operations such as remote connection as well as sending movement commands.
                 - Programs are sent to the 10x000 port (first robot port: 101000). 
                 - A program sent to robot will be executed immediately unless there is a prior operation running on the robot. 
                 - If a second motion command is sent while the referenced robot is moving, the second command is blocked and will not reply until the first motion is complete.
                 - Blended motion tolerance can be adjusted in the motion profile

    Serial Communication Messages from the Robot:
                 - Responses begin with a "0" if the command was successful, or a negative error code number

    """
    def __init__(self, robot_ID:str, host: str, port: int):
        self.logger = logging.getLogger("PF400_Client")
        self.logger.addHandler(logging.StreamHandler())
        
        self.ID = robot_ID
        self.host = host
        self.port = port
        
        # Default Motion Profile Paramiters
        self.motion_profile = {"Speed": 50, "Speed2": 0, "Acceleration": 100, "Deceleration": 100, "AccelRamp": 0.1, "DecelRamp": 0.1,"Straight": False, "InRange": 10}
        
        # Predefined locations for plate transferring oparetions
        self.location_dictionary = {"HomeALL": [-493.56371,-82.630104,1157.130034,-83.036521,90,180], 
                                    "HomeArm": [0,0,0,0,0,0], 
                                    "OT2_1_plate_rack": [1,0,0,0,0,0],
                                    "OT2_2_plate_rack": [2,0,0,0,0,0], 
                                    "OT2_3_plate_rack": [3,0,0,0,0,0], 
                                    "OT2_4_plate_rack": [4,0,0,0,0,0],
                                    "OT2_1_above_plate": [1,0,0,0,0,0],
                                    "OT2_2_above_plate": [2,0,0,0,0,0], 
                                    "OT2_3_above_plate": [3,0,0,0,0,0], 
                                    "OT2_4_above_plate": [4,0,0,0,0,0],
                                    "OT2_1_approach_plate": [1,0,0,0,0,0],
                                    "OT2_2_approach_plate": [2,0,0,0,0,0], 
                                    "OT2_3_approach_plate": [3,0,0,0,0,0], 
                                    "OT2_4_approach_plate": [4,0,0,0,0,0],
                                    "OT2_1_pick_plate": [1,0,0,0,0,0],
                                    "OT2_2_pick_plate": [2,0,0,0,0,0], 
                                    "OT2_3_pick_plate": [3,0,0,0,0,0], 
                                    "OT2_4_pick_plate": [4,0,0,0,0,0],
                                    "OT2_1_front": [1,0,0,0,0,0],
                                    "OT2_2_front": [2,0,0,0,0,0], 
                                    "OT2_3_front": [3,0,0,0,0,0], 
                                    "OT2_4_front": [4,0,0,0,0,0],
                                    "Transfer_A": [0,0,0,0,0,0],
                                    "Transfer_B": [0,0,0,0,0,0],
                                    "Transfer_C": [0,0,0,0,0,0],
                                    "Transfer_D": [0,0,0,0,0,0],
                                    "Table_rack": [0,0,0,0,0,0],
                                    "Mobile_robot": [0,0,0,0,0,0],
                                    "Successful_plate_transfer": [0,0,0,0,0,0], 
                                    "Trash": [0,0,0,0,0,0]
                                    }

        self.logger.info("Robot created. Robot ID: {} - Host: {} - Port: {}".format(self.ID, self.host, self.port))

    #Connect the socket object to the robot
    def connect_robot(self):    
        #create an INET, Streaming socket (IPv4, TCP/IP)
        try:
            PF400 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  

        except socket.error:
            self.logger.error('Failed to create socket')

        else:
            PF400.connect((self.host,self.port))
            # self.logger.info('Socket Created')
            return PF400     

        #self.logger.info('Socket Connected to ' + self.host + " Port:", self.port )
        #TODO:Read the status of the robot


    def disconnect_robot(self, PF400):
        PF400.close()
        # self.logger.info("TCP/IP client is closed")

    def enable_power(self):

        PF400 = self.connect_robot()

        #Send cmd to Activate the robot
        command = 'hp 1\n'
        # Add ASCII NULL character at the end of the cmd string
        try:
            PF400.send(bytes(command.encode('ascii')))
            out_msg = PF400.recv(4096).decode("utf-8")
            self.logger.info(out_msg)
            self.logger.info("Enabling power on the robot")

        except socket.error as err:
            self.logger.error('Failed enable_power: {}'.format(err))

        else:
            self.disconnect_robot(PF400)    

    def attach_robot(self):
        PF400 = self.connect_robot()

        command = 'attach 1\n'
        # Add ASCII NULL character at the end of the cmd string
        try:
            PF400.send(bytes(command.encode('ascii')))
            # time.sleep(15)
            out_msg = PF400.recv(4096).decode("utf-8")
            self.logger.info(out_msg)
            self.logger.info("Attaching the robot")

        except socket.error as err:
            self.logger.error('Failed to attach the robot: {}'.format(err))

        else:
            self.disconnect_robot(PF400)    
        
    def home_robot(self):

        PF400 = self.connect_robot()

        command = 'home\n'
        # Add ASCII NULL character at the end of the cmd string
        try:
            PF400.send(bytes(command.encode('ascii')))
            out_msg = PF400.recv(4096).decode("utf-8")
            self.logger.info(out_msg)
            self.logger.info("Homing the robot")

        except socket.error as err:
            self.logger.error('Failed to home the robot: {}'.format(err))
        else:
            self.disconnect_robot(PF400)  

    def initialize_robot(self):
        
        # Enable power 
        self.enable_power()
        # Attach robot
        self.attach_robot()
        # Home robot
        self.home_robot()
        # Set default motion profile
        self.set_profile()

        self.check_robot_state()

        self.logger.info("Robot initialization is successfully completed!")
    
    def set_motion_blend_tolerance(self, tolerance: int = 0):
        """
        Description: Gets or sets the InRange property of the selected profile, which is a parameter to set tolorance between blended motions.
        
        Paramiters
                tolerance:  from -1 to 100
                            -1 means do not stop at the end of motion if blending is possible.  
                            0 means always stop but do not check the end point error.  
                            > 0 means wait until close to the end point.  
                            Larger numbers mean less position error is allowed.
        """
        pass

    def check_robot_heartbeat(self):

        PF400 = self.connect_robot()

        command = 'nop\n'
        try:
            PF400.send(bytes(command.encode('ascii')))
            out_msg = PF400.recv(4096).decode("utf-8")
            self.logger.info(out_msg)

            if out_msg != None:
                self.logger.info("Robot is alive")
            else:
                self.logger.warning("Lost robot heartbeat")

        except socket.error as err:
            self.logger.error('Failed to check robot heartbeat: {}'.format(err))
        else:
            self.disconnect_robot(PF400)  

    def check_robot_state(self):

        PF400 = self.connect_robot()

        command = 'sysState\n'
        try:
            PF400.send(bytes(command.encode('ascii')))
            robot_state = PF400.recv(4096).decode("utf-8")
            self.logger.info(robot_state)

        except socket.error as err:
            self.logger.error('Failed to check robot state: {}'.format(err))

        else:
            self.disconnect_robot(PF400)  
            return robot_state

    def set_profile(self, profile_dict = {"0":0}):

        if len(profile_dict) == 1:
           
            self.logger.info("Setting defult values to the motion profile")

            PF400 = self.connect_robot()
            command = 'Profile 1'
            for key, value in self.motion_profile.items():
                command += ' ' + str(value)
            command += '\n'
            
            try:
                PF400.send(bytes(command.encode('ascii')))
                robot_state = PF400.recv(4096).decode("utf-8")
                self.logger.info(robot_state)

            except socket.error as err:
                self.logger.error('Failed to set motion profile: {}'.format(err))

            else:
                self.disconnect_robot(PF400)  


        elif len(profile_dict) == 8:

            self.logger.info("Setting new values to the motion profile")

            PF400 = self.connect_robot()
            command = 'Profile 1'
            for key, value in profile_dict.items():
                command += ' ' + str(value)
            command += '\n'
            
            try:
                PF400.send(bytes(command.encode('ascii')))
                robot_state = PF400.recv(4096).decode("utf-8")
                self.logger.info(robot_state)

            except socket.error as err:
                self.logger.error('Failed to set motion profile: {}'.format(err))

            else:
                self.disconnect_robot(PF400)  
        
        else:
            raise Exception("Motion profile takes 8 arguments, {} where given".format(len(profile_dict)))

    def wait_before_next_move(self):
        """
        If you want to wait for the robot to stop moving, issue a waitForEom command
        """
        pass
    
    def set_move_command(self, robot_location, grab: bool = False, release: bool = False):
        # TODO: FIND THE 5th JOINT VALUE FOR WHEN THE GRABBER IS CLOSE AND OPEN
        robot_command = "MoveJ 1" 

        for count, location in enumerate(self.location_dictionary[robot_location]):
            if grab == True and count == 4:
                robot_command += " 222"
            elif release == True and count == 4:
                robot_command += " 227"
            else:    
                robot_command += " " + str(location) 
        robot_command += '\n'
        
        return robot_command

    def pick_plate_ot2(self, ot2_ID):
        

        # OT2_4_above_plate
        # OT2_2_pick_plate
        # OT2_3_front
        # OT2_2_plate_rack

        # self.logger.info("Setting defult values to the motion profile")

        # Set movement commands to complete a pick_plate_ot2 operation
        move_front = self.set_move_command("OT2_" + str(ot2_ID) + "_front")
        above_plate = self.set_move_command("OT2_" + str(ot2_ID) + "_above_plate")
        approach_plate = self.set_move_command("OT2_" + str(ot2_ID) + "_pick_plate")
        pick_up_plate = self.set_move_command("OT2_" + str(ot2_ID) + "_pick_plate", True, False)
        above_with_plate = self.set_move_command("OT2_" + str(ot2_ID) + "_above_plate", True, False)
        front_with_plate = self.set_move_command("OT2_" + str(ot2_ID) + "_front", True, False)


        pick_up_commands = [move_front, above_plate, approach_plate, pick_up_plate, above_with_plate, front_with_plate]

        for count, command in enumerate(pick_up_commands):

            PF400 = self.connect_robot()
            try:
                PF400.send(bytes(command.encode('ascii')))
                out_msg = PF400.recv(4096).decode("utf-8")
                self.logger.info(out_msg)

                # TODO: CHECK FOR ERROR RETURN FORM ROBOT FIRST 
                self.logger.info("[pick_plate_ot2 ID:{}] Robot is moved to the {}th location".format(str(ot2_ID), count+1))

            except socket.error as err:
                self.logger.error('Failed move the robot {}'.format(err))
            else:
                self.disconnect_robot(PF400)  


    def drop_plate_ot2(self, ot2_ID):

        # Set movement commands to complete a drop_plate_ot2 operation
        front_with_plate = self.set_move_command("OT2_" + str(ot2_ID) + "_front", True, False)
        above_with_plate = self.set_move_command("OT2_" + str(ot2_ID) + "_above_plate", True, False)
        approach_with_plate = self.set_move_command("OT2_" + str(ot2_ID) + "_pick_plate", True, False)
        drop_plate = self.set_move_command("OT2_" + str(ot2_ID) + "_pick_plate", False, True)
        above_plate = self.set_move_command("OT2_" + str(ot2_ID) + "_above_plate")
        front_plate = self.set_move_command("OT2_" + str(ot2_ID) + "_front")


        drop_commands = [front_with_plate, above_with_plate, approach_with_plate, drop_plate, above_plate, front_plate]

        for count, command in enumerate(drop_commands):

            PF400 = self.connect_robot()
            try:
                PF400.send(bytes(command.encode('ascii')))
                out_msg = PF400.recv(4096).decode("utf-8")
                self.logger.info(out_msg)

                # TODO: CHECK FOR ERROR RETURN FORM ROBOT FIRST 
                self.logger.info("[pick_plate_ot2 ID:{}] Robot is moved to the {}th location".format(str(ot2_ID), count+1))

            except socket.error as err:
                self.logger.error('Failed move the robot {}'.format(err))
            else:
                self.disconnect_robot(PF400)  


    def program_robot_target(self, job:str, robot_ID_list: list):
        """
            Programs the robot to execute sequance of movements from its' current location to a given target location
        """
        # TODO: Find current robot pose
        # TODO: Find robot location in the operation environment and plan movements dependiging on the surrounding obstacles
        # TODO: HOME the robot arm before starting a program & Plan different movements dependng of the current grabber state

        if job.upper() == "TRANSFER":
            
            self.logger.info("Executing plate transfer between OT2 ID: {} and OT2 ID: {}".format(robot_ID_list[0],robot_ID_list[1]))
            self.pick_plate_ot2(robot_ID_list[0])
            self.drop_plate_ot2(robot_ID_list[1])

    def move_single(self, target_location):
        """
            Executes only one movement to the target location which is from the location dictionary
        """
        PF400 = self.connect_robot()
        command = "MoveJ 1"

        for location in self.location_dictionary[target_location]:
            command += " " + str(location) 
        command += '\n'

        try:
            PF400.send(bytes(command.encode('ascii')))
            out_msg = PF400.recv(4096).decode("utf-8")
            self.logger.info(out_msg)

            # TODO: CHECK FOR ERROR RETURN FORM ROBOT FIRST 
            self.logger.info("Robot is moved to the {} location".format(target_location))

        except socket.error as err:
            self.logger.error('Failed move the robot {}'.format(err))
        else:
            self.disconnect_robot(PF400)  

    def manualy_move_cartesian(self, target_joint):
        """
        A Cartesian location specifies the coordinates of a position in space using X, Y, and Z coordinates, and an orientation of the robot tool using yaw, pitch, and roll angles.
        Depending on the robot kinematics, there may be more than one set of joint angles that puts the robot's gripper at the same Cartesian location.
        """
        pass

    def manualy_move_joints(self, target_joint_location):
        """
        An angles location is a collection of the joint angles for the robot.  
        Joint angles are in units of degrees for rotary axes and in millimeters for linear axes.  
        By moving all joints to the specified values, the robot moves to an unambiguous position and orientation in space.
        """

        pass

    def set_speed(self, speed):

        PF400 = self.connect_robot()

        command = "mspeed" + str(speed) +"\n"
        try:
            PF400.send(bytes(command.encode('ascii')))
            out_msg = PF400.recv(4096).decode("utf-8")
            self.logger.info(out_msg)
        except socket.error as err:
            self.logger.error('Failed to send data: {}'.format(err))
        else:
            self.disconnect_robot(PF400) 

    def locate_robot(self):
        
        PF400 = self.connect_robot()
        command = 'where\n'
        try:
            PF400.send(bytes(command.encode('ascii')))
            # data = bytearray()
            # while True:
            #     packet = PF400.recv(4096)
            #     if not packet:
            #         break
            #     data.extend(packet)
            out_msg = PF400.recv(8000).decode("utf-8")
        except socket.error as err:
            print('Failed to send data')

        self.disconnect_robot(PF400)

        return out_msg



if __name__ == "__main__":
    robot = PF400("1","192.168.1.81",10000)
    robot.initialize_robot()
    # robot.move_single("HomeALL")
    # robot.pick_plate_ot2(1)
    # robot.drop_plate_ot2(2)
    robot.program_robot_target("Transfer",[4,3])
