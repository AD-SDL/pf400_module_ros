import os.path
import time
import logging

import zmq
import struct

from pf400_client import PF400

#Log Configuration
file_path = os.path.join(os.path.split(os.path.dirname(__file__))[0]  + '/pf400_logs/ros_listener_logs.log')

logging.basicConfig(filename = file_path, level=logging.DEBUG, format = '[%(levelname)s] [%(asctime)s] [%(name)s] %(message)s', datefmt = '%Y-%m-%d %H:%M:%S')

class RPL_PF400(PF400):
    """
    Python interface to socket interface of  PF400.
    Listens for messges come from ROS Arm Node.

    """
    def __init__(self):

        self.logger = logging.getLogger("PF400_listen")
        self.logger.addHandler(logging.StreamHandler())
 
        self.OT2_ID = {"bob":1, "alex":2, "jack":3}

        self.data_file = 'robot_data.json'
        self.rpl_robot = PF400(self.data_file)
        self.print_robot()

    def print_robot(self):
        print(self.rpl_robot.location_dictionary)

    def is_robot_running(self, msg):
        # Cehck heartbeat and state of the robot regurly                                                                                
        output = self.rpl_robot.initialize_robot()

    def command_handler(self, msg):
        self.rpl_robot.set_robot_mode()
        msg = msg.split("@")

        # Check robot state 
        # while self.rpl_robot.check_general_state() == -1:

        #     self.logger.warn("Robot is not intilized! Intilizing now...")
        #     output = robot.initialize_robot()

        if len(msg) == 3 and msg[0].lower() == "transfer":
            output = self.program_rpl_robot(msg[0],self.OT2_ID[msg[1]],self.OT2_ID[msg[2]])
        elif len(msg) == 2 and msg[0].lower() == "rack":
            output = self.pick_plate_from_rack(self.OT2_ID[msg[1]])
        elif len(msg) == 1 and msg[0].lower() == "complete":
            output = robot.drop_complete_plate()
        else:
            self.logger.error("User sent invalid command")
            return "Invalid command requested by the client!!!"    

        return output
    
    def pick_plate_ot2(self, ot2_ID , profile = 0, wait:int = 0.1):
        
        # TODO:ADD Motion profile index
        
        if profile == 3: 
            slow, fast = 3, 3
        else:
            slow, fast = 1, 2
            

        # self.logger.info("Setting defult values to the motion profile")

        # Set movement commands to complete a pick_plate_ot2 operation
        move_front = self.rpl_robot.set_move_command("ot2_" + str(ot2_ID) + "_front",fast)
        above_plate = self.rpl_robot.set_move_command("ot2_" + str(ot2_ID) + "_above_plate",fast)
        approach_plate = self.rpl_robot.set_move_command("ot2_" + str(ot2_ID) + "_pick_plate",fast)
        pick_up_plate = self.rpl_robot.set_move_command("ot2_" + str(ot2_ID) + "_pick_plate", slow, True, False)
        above_with_plate = self.rpl_robot.set_move_command("ot2_" + str(ot2_ID) + "_above_plate", slow, True, False)
        front_with_plate = self.rpl_robot.set_move_command("ot2_" + str(ot2_ID) + "_front", slow, True, False)

        pick_up_commands = [move_front, above_plate, approach_plate, pick_up_plate, above_with_plate, front_with_plate] 

        for count, cmd in enumerate(pick_up_commands):
                   
            input_msg = "[pick_plate_ot2 ID:{}] Robot is moved to the {}th location".format(str(ot2_ID), count+1)
            err_msg = 'Failed move the robot:'

            out_msg = self.rpl_robot.send_command(cmd, input_msg, err_msg, wait)
              
        return out_msg


    def drop_plate_ot2(self, ot2_ID, profile = 0, wait:int = 0.1):

        # Set movement commands to complete a drop_plate_ot2 operation
        if profile == 3: 
            slow, fast = 3, 3
        else:
            slow, fast = 1, 2
            

        front_with_plate = self.rpl_robot.set_move_command("ot2_" + str(ot2_ID) + "_front", slow, True, False)
        above_with_plate = self.rpl_robot.set_move_command("ot2_" + str(ot2_ID) + "_above_plate", slow, True, False)
        approach_with_plate = self.rpl_robot.set_move_command("ot2_" + str(ot2_ID) + "_pick_plate", slow, True, False)
        drop_plate = self.rpl_robot.set_move_command("ot2_" + str(ot2_ID) + "_pick_plate", slow, False, True)
        above_plate = self.rpl_robot.set_move_command("ot2_" + str(ot2_ID) + "_above_plate", fast)
        front_plate = self.rpl_robot.set_move_command("ot2_" + str(ot2_ID) + "_front", fast)


        drop_to_ot2 = [front_with_plate, above_with_plate, approach_with_plate, drop_plate, above_plate, front_plate]
        
        for count, cmd in enumerate(drop_to_ot2):

            input_msg = "[drop_plate_ot2 ID:{}] Robot is moved to the {}th location".format(str(ot2_ID), count+1)
            err_msg = 'Failed move the robot:'

            out_msg = self.rpl_robot.send_command(cmd, input_msg, err_msg, wait)
              
        return out_msg
                
    def pick_plate_from_rack(self, ot2_ID, profile = 0, wait:int = 0.1):
        
        if profile == 3: 
            slow, fast = 3, 3
        else:
            slow, fast = 1, 2

        approach_plate_rack = self.rpl_robot.set_move_command("ot2_" + str(ot2_ID) + "_approach_plate_rack", fast, False, False)
        front_rack = self.rpl_robot.set_move_command("ot2_" + str(ot2_ID) + "_front_plate_rack", slow, False, False)
        plate_rack = self.rpl_robot.set_move_command("ot2_" + str(ot2_ID) + "_plate_rack", slow, False, False)
        pick_plate = self.rpl_robot.set_move_command("ot2_" + str(ot2_ID) + "_plate_rack", slow, True, False)
        front_with_plate = self.rpl_robot.set_move_command("ot2_" + str(ot2_ID) + "_front_plate_rack", slow, True, False)
        approach_rack_back = self.rpl_robot.set_move_command("ot2_" + str(ot2_ID) + "_approach_plate_rack", slow, True, False)

        pick_plate_rack = [approach_plate_rack, front_rack, plate_rack, pick_plate, front_with_plate, approach_rack_back]
       
        for count, cmd in enumerate(pick_plate_rack):
                   
            input_msg = "[pick_plate_from_rack ID:{}] Robot is moved to the {}th location".format(str(ot2_ID), count+1)
            err_msg = 'Failed move the robot:'

            out_msg = self.rpl_robot.send_command(cmd, input_msg, err_msg, wait)
              
        return out_msg
        

    def drop_complete_plate(self, profile = 0, wait:int = 0.1):
        
        if profile == 3: 
            slow, fast = 3, 3
        else:
            slow, fast = 1, 2

        completed_plate_above = self.rpl_robot.set_move_command("completed_plate_above", slow, True, False)
        drop_with_plate = self.rpl_robot.set_move_command("completed_plate", slow, True, False)
        drop_plate = self.rpl_robot.set_move_command("completed_plate", slow, False, True)
        completed_plate = self.rpl_robot.set_move_command("completed_plate_above", slow, False, False)
       

        complete_plate = [completed_plate_above, drop_with_plate, drop_plate, completed_plate]
       
        for count, cmd in enumerate(complete_plate):
                   
            input_msg = "[drop_complete_plate] Robot is moved to the {}th location".format(count+1)
            err_msg = 'Failed move the robot:'

            out_msg = self.rpl_robot.send_command(cmd, input_msg, err_msg, wait)
              
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
            self.rpl_robot.move_single("homeall", 2)
            self.pick_plate_ot2(robot_ID_1)
            self.drop_plate_ot2(robot_ID_2)
            

        elif job.upper() == "FULL_TRANSFER":

            self.logger.info("Executing full transfer")
            self.rpl_robot.move_single("homeall", 2)
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

   
if __name__ == "__main__":
    robot = RPL_PF400()
    robot.command_handler("rack@bob")
