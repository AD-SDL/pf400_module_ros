import os.path
import time
import logging
import json

from pf400_client import PF400

#Log Configuration
file_path = os.path.join(os.path.split(os.path.dirname(__file__))[0]  + '/pf400_logs/rpl_pf400_logs.log')

logging.basicConfig(filename = file_path, level=logging.DEBUG, format = '[%(levelname)s] [%(asctime)s] [%(name)s] %(message)s', datefmt = '%Y-%m-%d %H:%M:%S')

class RPL_PF400(PF400):
    """
    Python interface to socket interface of  PF400.
    Listens for messges come from ROS Arm Node.

    """
    def __init__(self, data_file_path = "robot_data.json"):
        super().__init__(data_file_path)
 
        self.OT2_ID = {"bob":1, "alex":2, "jack":3}

    def command_handler(self, msg):
        msg = msg.split("@")

        self.force_initialize_robot()

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

    def rpl_teach_location(self, location:str = None):

        output = self.disable_power()
        if output[0]  == "-":
            raise Exception("Falied disabling power. Aborting teach location!")

        self.logger.info("Please set the robot location by manually moving the arm to the disered location.\n" \
            "!!! ------------------------------------ IMPORTANT ------------------------------------ !!! \n" \
            "To release the vertical rail, HOLD the arm to make sure it does not fall down and " \
            "then press the black release button underneath the second joint.")

        user_save_input = str(input("Do you want to save the current location? (y/n): "))
        
        if user_save_input.lower() == "y":
            output = self.rpl_save_location(location)
            self.logger.info(output)
        elif user_save_input.lower() == "n":
            self.logger.warning("Location is not saved!")
        else:
            self.logger.warning("Please enter 'y' to save or 'n' to not save the new location")
            return self.rpl_teach_location(location)
        
        self.force_initialize_robot()

    def rpl_save_location(self, location:str = None):

        if location == None:
            self.logger.error("[rpl_save_location] Location name was not provided by the user")
            raise Exception("Please enter a location name to save the new joint values")

        location = location.lower()
        # TODO: Check if location exist in the robot data
        if self.check_loc_data(location) == False:
            return "Invalid location entered"
        
        # Setting parent file directory 
        current_directory = os.path.dirname(__file__)
        parent_directory = os.path.split(current_directory)[0] 
        file_path = os.path.join(parent_directory + '/utils/robot_data.json')
        
        current_location = self.locate_robot()

        # Save the current robot location to the given location
        loc_list = list(map(float,current_location.split(" ")))
        self.location_dictionary[location] = [loc_list[1], loc_list[2], loc_list[3], loc_list[4], loc_list[5], loc_list[6]]
    
        # Write the new location the data file
        try:
            with open(file_path, "w") as jsonFile:
                json.dump(self.robot_data, jsonFile, indent=4)
        except IOError as err:
            self.logger.error(err)

        return "Current location is saved to {}".format(location)
    
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
            time.sleep(5)
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
   
if __name__ == "__main__":
    robot = RPL_PF400()
    # robot.command_handler("rack@bob")
    robot.rpl_teach_location("transfer_d")
