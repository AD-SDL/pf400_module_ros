import socket
import time
import logging

#Log Configuration
logging.basicConfig(filename = "robot_client_logs.log", level=logging.DEBUG, format = '[%(levelname)s] [%(name)s] %(message)s')
# Arm_Robot object is the main object that will be used for operations such as remote connection as well as sending movement commands
class arm_robot(object):
    """
    Description: Python interface that allows remote commands to be executed using simple string messages over TCP/IP on PF400 cobot. 
                 Arm_Robot is the main object that will be used for operations such as remote connection as well as sending movement commands.
                 Programs are sent to the 10x000 port (first robot port: 101000). A program sent to robot will be executed immediately. 
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
                                    "OT2_1_plate_rack": [0,0,0,0,0,0],
                                    "OT2_2_plate_rack": [0,0,0,0,0,0], 
                                    "OT2_3_plate_rack": [0,0,0,0,0,0], 
                                    "OT2_4_plate_rack": [0,0,0,0,0,0],
                                    "OT2_1_above_plate": [0,0,0,0,0,0],
                                    "OT2_2_above_plate": [0,0,0,0,0,0], 
                                    "OT2_3_above_plate": [0,0,0,0,0,0], 
                                    "OT2_4_above_plate": [0,0,0,0,0,0],
                                    "OT2_1_approach_plate": [0,0,0,0,0,0],
                                    "OT2_2_approach_plate": [0,0,0,0,0,0], 
                                    "OT2_3_approach_plate": [0,0,0,0,0,0], 
                                    "OT2_4_approach_plate": [0,0,0,0,0,0],
                                    "OT2_1_pick_plate": [0,0,0,0,0,0],
                                    "OT2_2_pick_plate": [0,0,0,0,0,0], 
                                    "OT2_3_pick_plate": [0,0,0,0,0,0], 
                                    "OT2_4_pick_plate": [0,0,0,0,0,0],
                                    "OT2_1_front": [0,0,0,0,0,0],
                                    "OT2_2_front": [0,0,0,0,0,0], 
                                    "OT2_3_front": [0,0,0,0,0,0], 
                                    "OT2_4_front": [0,0,0,0,0,0],
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
            self.logger.info('Socket Created')
            return PF400     

        #self.logger.info('Socket Connected to ' + self.host + " Port:", self.port )
        #TODO:Read the status of the robot


    def disconnect_robot(self, PF400):
        PF400.close()
        self.logger.info("TCP/IP client is closed")

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

        self.logger.info("Robot initialization is successfully completed!")

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



    def program_robot_target(self, target_location):
        """
            Programs the robot to execute sequance of movements from its' current location to a given target location
        """
        pass 

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

        pass

    def manualy_move_joints(self, target_joint_location):

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
    robot = arm_robot("1","192.168.1.81",10000)
    robot.initialize_robot()
    robot.move_single("HomeALL")
    
