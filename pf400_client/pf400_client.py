import os.path
import socket
import time
import logging
import json

#Log Configuration
file_path = os.path.join(os.path.split(os.path.dirname(__file__))[0]  + '/pf400_logs/robot_client_logs.log')

logging.basicConfig(filename = file_path, level=logging.DEBUG, format = '[%(levelname)s] [%(asctime)s] [%(name)s] %(message)s', datefmt = '%Y-%m-%d %H:%M:%S')

class PF400(object):
    """
    Description: 
                 - Python interface that allows remote commands to be executed using simple string messages over TCP/IP on PF400 cobot. 
                 - PF400 is the main object that will be used for operations such as remote connection as well as sending movement commands.
                 - Programs are sent to the 10x00 port (first robot port: 10100). 
                 - A program sent to robot will be executed immediately unless there is a prior operation running on the robot. 
                 - If a second motion command is sent while the referenced robot is moving, the second command is blocked and will not reply until the first motion is complete.
                 - Blended motion tolerance can be adjusted in the motion profile

    Serial Communication Messages from the Robot:
                 - Responses begin with a "0" if the command was successful, or a negative error code number

    """
    def __init__(self):
        
        self.logger = logging.getLogger("PF400_Client")
        self.logger.addHandler(logging.StreamHandler())
        
        robot_data, robot1, motion_profile, locations = self.load_robot_data()
        self.ID = robot1["id"]
        self.host = robot1["host"]
        self.port = robot1["port"]
        self.robot_data = robot_data
       
        
        # Default Motion Profile Paramiters. Using two profiles for faster and slower movements
        self.motion_profile = motion_profile
        # TODO: Use second motion prfile for slower movements 

        # Predefined locations for plate transferring oparetions
        self.location_dictionary = locations

        self.logger.info("Robot created. Robot ID: {} ~ Host: {} ~ Port: {}".format(self.ID, self.host, self.port))
    
    def load_robot_data(self):
        # Setting parent file directory 
        current_directory = os.path.dirname(__file__)
        parent_directory = os.path.split(current_directory)[0] 
        file_path = os.path.join(parent_directory + '/utils/robot_data.json')

        # load json file
        with open(file_path) as f:
            data = json.load(f)

        f.close()

        return data, data["robot_data"][0], data["robot_data"][0]["motion_profile"],data["robot_data"][0]["locations"][0]


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

    def send_command(self, cmd: str=None, ini_msg:str = None, err_msg:str = None, wait:int = 0.1):
        """
        Send and arbitrary command to the robot
        - command : 
        - wait : wait time after movement is complete
        """

        ##Command Checking 
        #TODO: We can check the available commands if the user enters a wrong one break

        if cmd==None:
            self.logger.info("Invalid command: " +cmd)
            return -1 ## make it return the last valid state
        
        ##logging messages
        if ini_msg=='':
            ini_msg = 'send command'
        if err_msg=='':
            err_msg = 'Failed to send command: '

        ##TODO check cmd against cmd list 
        ##return invalid CMD before trying to connect

        PF400_sock = self.connect_robot()
        
        try:
            PF400_sock.send(bytes(cmd.encode('ascii')))
            robot_output = PF400_sock.recv(4096).decode("utf-8")
            if ini_msg:
                self.logger.info(ini_msg)
            self.logger.info(robot_output)
            # Wait after executing the command. Default wait time 0.1 sc
            time.sleep(wait)
        except socket.error as err:
            self.logger.error(err_msg +' {}'.format(err))

            return('failed')## what is a failed state or it is the last state
        else:
            self.disconnect_robot(PF400_sock)
            # Returning the output message as a list             
            return(robot_output)


    def check_robot_state(self, wait:int = 0.1):

        cmd = 'sysState\n'
        input_msg = 'Checking robot state:'
        err_msg = 'Failed to check robot state:'

        out_msg = self.send_command(cmd, input_msg, err_msg)
        return out_msg



    def enable_power(self, wait:int = 0.1):

        #Send cmd to Activate the robot
        cmd = 'hp 1\n'
        ini_msg = 'Enabling power on the robot'
        err_msg = 'Failed enable_power:'

        out_msg = self.send_command(cmd, ini_msg, err_msg, wait)

        return out_msg


    def attach_robot(self, wait:int = 0.1):

        cmd = 'attach 1\n'
        ini_msg = "Attaching the robot"
        err_msg = "Failed to attach the robot:"

        out_msg = self.send_command(cmd, ini_msg, err_msg, wait)

        return out_msg

        
    def home_robot(self, wait:int = 0.1):

        cmd = 'home\n'
        ini_msg = 'Homing the robot'
        err_msg = 'Failed to home the robot: '

        out_msg = self.send_command(cmd, ini_msg, err_msg, wait)

        return out_msg

    
    def initialize_robot(self):
        
        # Enable power 
        power = self.enable_power(5)
        # Attach robot
        attach = self.attach_robot(5)
        # Home robot
        home = self.home_robot(15)
        # Set default motion profile
        profile = self.set_profile(5)
        # Check robots' current state
        rState =self.check_robot_state()

        if power[0].find('-') == -1 and attach[0].find('-') == -1 and home[0].find('-') == -1 and profile[0].find('-')== -1 :
            self.logger.info("Robot initialization is successfully completed!")
        else:    
            self.logger.info("Robot initialization failed!")

        return power + attach + profile + home + rState

        
    ##create "profile section" apart from the "command section"
    def set_profile(self, wait:int = 0.1, profile_dict:dict = {"0":0}):

        if len(profile_dict) == 1:
           
            cmd = 'Profile 1'
            for key, value in self.motion_profile[0].items():
                cmd += ' ' + str(value)
            cmd += '\n'

            cmd2 = 'Profile 2'
            for key, value in self.motion_profile[1].items():
                cmd2 += ' ' + str(value)
            cmd2 += '\n'

            ini_msg = "Setting defult values to the motion profile 1"
            ini_msg2 = "Setting defult values to the motion profile 2"
            err_msg = 'Failed to set profile 1: '
            err_msg2 = 'Failed to set profile 2: '

            out_msg = self.send_command(cmd, ini_msg, err_msg, wait)
            out_msg2 = self.send_command(cmd2, ini_msg2, err_msg2, wait)



        elif len(profile_dict) == 8:

            ini_msg = "Setting new values to the motion profile 3"
            err_msg = 'Failed to set profile 1: '

            cmd = 'Profile 3'
            for key, value in profile_dict.items():
                cmd += ' ' + str(value)
            cmd += '\n'

            out_msg = self.send_command(cmd, ini_msg, err_msg, wait)
            
        else:
            raise Exception("Motion profile takes 8 arguments, {} where given".format(len(profile_dict)))

        return out_msg 



    
    def set_motion_blend_tolerance(self, tolerance: int = 0, wait:int = 0.1):
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

    def set_robot_mode(self):
        
        cmd = 'mode 0\n'
        
        input_msg = 'Setting communication mode to 0:'
        err_msg = 'Failed to set communication mode to 0'

        out_msg = self.send_command(cmd, input_msg, err_msg)
        return out_msg

    def check_robot_heartbeat(self, wait:int = 0.1):

        cmd = 'nop\n'
        
        input_msg = 'Checking robot heartbeat:'
        err_msg = 'Failed to check robot heartbeat:'

        out_msg = self.send_command(cmd, input_msg, err_msg, wait)
        return out_msg

    def check_general_state(self, wait:int = 0.1):

        cmd1 = "hp\n"
        cmd2 = "attach\n"
        cmd3 = "sysState\n"

        power_msg = self.send_command(cmd1)
        power_msg = power_msg.split(" ")

        attach_msg = self.send_command(cmd2)
        attach_msg = attach_msg.split(" ")

        state_msg = self.send_command(cmd3)
        state_msg = state_msg.split(" ")

        power ,attach, state = 0, 0, 0

        if len(power_msg) == 1 or power_msg[0].find("-") != -1:
            power = -1
        if attach_msg[1].find("0") != -1 or attach_msg[0].find("-") != -1:
            attach = -1
        if state_msg[1].find("7") != -1 or state_msg[0].find("-") != -1:
            state = -1
        
        if power == -1 or attach == -1 or state == -1:
            return -1
        else: 
            return 0

    def stop_robot(self, wait:int = 0.1):
        """
        Stops the robot immediately but leaves power on.
        """

        cmd =  'halt\n'
        input_msg = 'Stopping robot:'
        err_msg = 'Failed to stop robot:'

        out_msg = self.send_command(cmd, input_msg, err_msg, wait)
        return out_msg        

    def wait_before_next_move(self, wait:int = 0.1):
        """
        If you want to wait for the robot to stop moving, issue a waitForEom command
        """
        pass

    def clear_programs(self, wait:int = 0.1):
        #TODO: CLEAR ROBOT MEMORY BEFORE STARTING A PROGRAM TO MAKE SURE THERE IS NO QUEUED PROGRAMS FROM PREVIOUS EXECUTION
        pass

    def set_move_command(self, robot_location, profile:int = 2, gripper: bool = False, release: bool = False):
        # TODO: FIND THE 5th JOINT VALUE FOR WHEN THE GRIPPER IS CLOSE AND OPEN
        if profile == 1:
            robot_command = "MoveJ 1" 

        elif profile == 2:
            robot_command = "MoveJ 2" 

        elif profile == 3:    
            robot_command = "MoveJ 3" 

        else:
            raise Exception("Please enter a valid motion profiile! 1 for slower movement, 2 for faster movement profile, 3 for modified profile")
       
        for count, location in enumerate(self.location_dictionary[robot_location]):
            if gripper == True and count == 4:
                robot_command += " " + str(120.0)
            elif release == True and count == 4:
                robot_command += " " + str(127.0)
            else:    
                robot_command += " " + str(location) 
        robot_command += '\n'
        
        return robot_command

    def pick_plate_ot2(self, ot2_ID , profile = 0, wait:int = 0.1):
        
        # TODO:ADD Motion profile index
        
        if profile == 3: 
            slow, fast = 3, 3
        else:
            slow, fast = 1, 2
            

        # self.logger.info("Setting defult values to the motion profile")

        # Set movement commands to complete a pick_plate_ot2 operation
        move_front = self.set_move_command("ot2_" + str(ot2_ID) + "_front",fast)
        above_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_above_plate",fast)
        approach_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_pick_plate",fast)
        pick_up_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_pick_plate", slow, True, False)
        above_with_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_above_plate", slow, True, False)
        front_with_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_front", slow, True, False)

        pick_up_commands = [move_front, above_plate, approach_plate, pick_up_plate, above_with_plate, front_with_plate] 

        for count, cmd in enumerate(pick_up_commands):
                   
            input_msg = "[pick_plate_ot2 ID:{}] Robot is moved to the {}th location".format(str(ot2_ID), count+1)
            err_msg = 'Failed move the robot:'

            out_msg = self.send_command(cmd, input_msg, err_msg, wait)
              
        return out_msg


    def drop_plate_ot2(self, ot2_ID, profile = 0, wait:int = 0.1):

        # Set movement commands to complete a drop_plate_ot2 operation
        if profile == 3: 
            slow, fast = 3, 3
        else:
            slow, fast = 1, 2
            

        front_with_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_front", slow, True, False)
        above_with_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_above_plate", slow, True, False)
        approach_with_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_pick_plate", slow, True, False)
        drop_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_pick_plate", slow, False, True)
        above_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_above_plate", fast)
        front_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_front", fast)


        drop_to_ot2 = [front_with_plate, above_with_plate, approach_with_plate, drop_plate, above_plate, front_plate]
        
        for count, cmd in enumerate(drop_to_ot2):

            input_msg = "[drop_plate_ot2 ID:{}] Robot is moved to the {}th location".format(str(ot2_ID), count+1)
            err_msg = 'Failed move the robot:'

            out_msg = self.send_command(cmd, input_msg, err_msg, wait)
              
        return out_msg
                
    def pick_plate_from_rack(self, ot2_ID, profile = 0, wait:int = 0.1):
        
        if profile == 3: 
            slow, fast = 3, 3
        else:
            slow, fast = 1, 2

        approach_plate_rack = self.set_move_command("ot2_" + str(ot2_ID) + "_approach_plate_rack", fast, False, False)
        front_rack = self.set_move_command("ot2_" + str(ot2_ID) + "_front_plate_rack", slow, False, False)
        plate_rack = self.set_move_command("ot2_" + str(ot2_ID) + "_plate_rack", slow, False, False)
        pick_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_plate_rack", slow, True, False)
        front_with_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_front_plate_rack", slow, True, False)
        approach_rack_back = self.set_move_command("ot2_" + str(ot2_ID) + "_approach_plate_rack", slow, True, False)

        pick_plate_rack = [approach_plate_rack, front_rack, plate_rack, pick_plate, front_with_plate, approach_rack_back]
       
        for count, cmd in enumerate(pick_plate_rack):
                   
            input_msg = "[pick_plate_from_rack ID:{}] Robot is moved to the {}th location".format(str(ot2_ID), count+1)
            err_msg = 'Failed move the robot:'

            out_msg = self.send_command(cmd, input_msg, err_msg, wait)
              
        return out_msg
        

    def drop_complete_plate(self, profile = 0, wait:int = 0.1):
        
        if profile == 3: 
            slow, fast = 3, 3
        else:
            slow, fast = 1, 2

        completed_plate_above = self.set_move_command("completed_plate_above", slow, True, False)
        drop_with_plate = self.set_move_command("completed_plate", slow, True, False)
        drop_plate = self.set_move_command("completed_plate", slow, False, True)
        completed_plate = self.set_move_command("completed_plate_above", slow, False, False)
       

        complete_plate = [completed_plate_above, drop_with_plate, drop_plate, completed_plate]
       
        for count, cmd in enumerate(complete_plate):
                   
            input_msg = "[drop_complete_plate] Robot is moved to the {}th location".format(count+1)
            err_msg = 'Failed move the robot:'

            out_msg = self.send_command(cmd, input_msg, err_msg, wait)
              
        return out_msg

    def move_single(self, target_location, profile = 2, grap: bool = False, release: bool = False, wait:int = 0.1):

        """
            Executes only one movement to the target location which is from the location dictionary
        """
        single_move_commands = self.set_move_command(target_location.lower(), profile, grap, release)

        input_msg = "Robot is moved to the {} location".format(target_location)
        err_msg = 'Failed move the robot:'

        out_msg = self.send_command(single_move_commands, input_msg, err_msg, wait)
            
        return out_msg


    def program_rpl_robot(self, job:str, robot_ID_1: int = 0, robot_ID_2:int = 0):
        """
            Programs the robot to execute sequance of movements from its' current location to a given target location
        """
        # TODO: Find current robot pose
        # TODO: Find robot location in the operation environment and plan movements dependiging on the surrounding obstacles
        # TODO: HOME the robot arm before starting a program & Plan different movements dependng of the current gripper state

        if job.upper() == "TRANSFER":

            self.logger.info("Executing plate transfer between OT2 ID: {} and OT2 ID: {}".format(robot_ID_1, robot_ID_2))
            self.move_single("homeall", 2)
            self.pick_plate_ot2(robot_ID_1)
            self.drop_plate_ot2(robot_ID_2)
            

        elif job.upper() == "FULL_TRANSFER":

            self.logger.info("Executing full transfer")
            self.move_single("homeall", 2)
            self.pick_plate_from_rack(1)
            self.drop_plate_ot2(1)
            time.sleep(50)
            self.pick_plate_ot2(1)
            self.drop_plate_ot2(2)
            time.sleep(50)
            self.pick_plate_ot2(2)
            self.drop_plate_ot2(3)
            time.sleep(50)
            self.pick_plate_ot2(3)
            self.drop_complete_plate()

            

    
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

    def set_speed(self, speed, wait:int = 0.1):

        cmd = "mspeed " + str(speed) +"\n"

        input_msg = "Settin new robot speed"
        err_msg = 'Failed set the robot speed:'

        out_msg = self.send_command(cmd, input_msg, err_msg, wait)
        return out_msg
            
    def locate_robot(self, wait:int = 0.1):
        
        location = 'wherej\n'

        input_msg = "Finding robot location:"
        err_msg = 'Failed to find robot location:'

        out_msg = self.send_command(location, input_msg, err_msg, wait)
        return out_msg
   
    def program_robot(self, location_list: list = []):
        """
        Description: A function to send multiple locations to the robot to execute them consecutively
        Params: - location_list: 2D array where first dimention contains location name from location data 
                                 and second contains gripper postion
        """       
        pass

    def check_loc_data(self, target_location):
        loc_list = list(self.location_dictionary.keys())
        try:
            idx = loc_list.index(target_location)
        except Exception:
            self.logger.error("Given location doesn't exist in the location data")
        else:
            return True
        finally:
            return False
        


    def rpl_teach_location(self, location:str = None):

        if location == None:
            self.logger.error("[rpl_teach_location] Location name was not provided by the user")
            raise Exception("Please enter a location name to save the new joint values")

        location = location.lower()
        # TODO: Check if location exist in the robot data
        if self.check_loc_data(location) == False:
            return "Invalid location entered"
        
        # Setting parent file directory 
        current_directory = os.path.dirname(__file__)
        parent_directory = os.path.split(current_directory)[0] 
        file_path = os.path.join(parent_directory + '/utils/robot_data.json')

        # Save the current robot location to the given location
        loc_list = list(map(int,"2 2 2 2 2 2".split(" ")))
        self.location_dictionary[location]= [loc_list[0],loc_list[1], loc_list[2], loc_list[3], loc_list[4], loc_list[5]]
    
        # Write the new location the data file
        try:
            with open(file_path, "w") as jsonFile:
                json.dump(self.robot_data, jsonFile, indent=4)
        except IOError as err:
            self.logger.error(err)

        return "Current location is saved to {}".format(location)
    
    

    def teach_new_location(self, location_name: str, gripper: str = 127):
        # A fuction to save any location data into the location data file with a given name by the user.
        locations = self.modify_robot_data()

        pass   

    def modify_robot_data(self, location: str, robot_id:int = None):

        current_location = self.locate_robot()
        location_data = self.modify_robot_data()

        try:
            if robot_id == None and (location.lower() == "homeall" or location.lower() == "homearm" or location.lower() == "mobile_robot" or location.lower() == "trash") :
            # TODO: Find a better way for this 
                for count, loc in enumerate(self.location_dictionary[location]):
                    self.location_dictionary[location][count] = current_location[count]

            elif robot_id != None:
                if location.upper() == "FRONT":
                    location_name = "ot2_" + str(robot_id) + "_front"
                elif location.upper() == "ABOVE_PLATE":
                    location_name =  "ot2_" + str(robot_id) + "_above_plate"
                elif location.upper() == "PICK_PLATE":
                    location_name = "ot2_" + str(robot_id) + "_pick_plate"
                elif location.upper() == "PLATE_RACK":
                    location_name = "ot2_" + str(robot_id) + "_plate_rack"
            else:
                raise Exception("Please enter a valid location name!!! Format: location: str, robot_id:int = None ")
            
            for count, loc in enumerate(self.location_dictionary[key_name]):
                self.location_dictionary[key_name][count] = current_location[count]        
           
        
        except Exception as err:
            self.logger.error(err)



if __name__ == "__main__":
    robot = PF400()
    ''' ---------------- TESTING -------------------'''
    robot.rpl_teach_location("Transfer_D")
    # robot.move_single("Transfer_D")
    # robot.updateJsonFile()
    # robot.move_single("HomeALL")
    # robot.set_speed(100)
    # robot.locate_robot()
    # robot.initialize_robot()
    # robot.pick_plate_from_rack(1)
    # robot.drop_complete_plate()
    # robot.pick_plate_ot2(1)
    # robot.load_robot_data()

    #TODO: Return out msg and error code



    # robot.pick_plate_from_rack(1)
    # robot.program_rpl_robot("full_transfer")
    # robot.move_single("HomeALL")
    # robot.pick_plate_ot2(1)
    # robot.drop_plate_ot2(2)
    # robot.teach_location("HomeALL")
    # robot.teach_location("above_plate",2)


    # try:
    #     while True:
    #         robot.program_robot_target("Transfer",[1,2])
    #         robot.program_robot_target("Transfer",[2,1])
    # except KeyboardInterrupt as err:
    #     print(err)
