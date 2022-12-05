#! /usr/bin/env python3

import rclpy                 # import Rospy
from rclpy.node import Node  # import Rospy Node
from std_msgs.msg import String
from std_srvs.srv import Empty

from time import sleep

from threading import Thread

from wei_services.srv import WeiDescription 
from wei_services.srv import WeiActions  

from pf400_driver.pf400_driver import PF400
from pf400_driver.pf400_camera_driver import PF400_CAMERA

class PF400ClientNode(Node):
    '''
    The jointControlNode inputs data from the 'action' topic, providing a set of commands for the driver to execute. It then receives feedback, 
    based on the executed command and publishes the state of the peeler and a description of the peeler to the respective topics.
    '''
    def __init__(self, NODE_NAME = "PF400_Client_Node"):
        '''
        The init function is neccesary for the peelerNode class to initialize all variables, parameters, and other functions.
        Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
        '''

        super().__init__(NODE_NAME)
        
        self.state = "UNKNOWN"

        self.pf400 = PF400("192.168.50.50", "10100")
        self.pf400.initialize_robot()
        self.get_logger().info("PF400 online")
        self.module_explorer = PF400_CAMERA(self.pf400)
        timer_period = 0.5  # seconds

        self.stateTimer = self.create_timer(timer_period, self.stateCallback)
        self.statePub = self.create_publisher(String, NODE_NAME + '/state', 10)
        # self.stateTimer = self.create_timer(timer_period, self.stateCallback)
        state_thread = Thread(target = self.stateCallback)
        state_thread.start()

        self.action_handler = self.create_service(WeiActions, NODE_NAME + "/action_handler", self.actionCallback)
        self.description_handler = self.create_service(WeiDescription, NODE_NAME + "/description_handler", self.descriptionCallback)

        self.description={}


    def stateCallback(self):
        '''
        Publishes the pf400 state to the 'state' topic. 
        '''
        state = self.pf400.movement_state
        if state == 0:
            self.state = "POWER OFF"
        elif state == 1:
            self.state = "READY"
        elif state == 2 or state == 3:
            self.state = "BUSY"
        msg = String()
        msg.data = 'State: %s' % self.state
        self.statePub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        sleep(0.5)
        

    def descriptionCallback(self, request, response):
        """The descriptionCallback function is a service that can be called to showcase the available actions a robot
        can preform as well as deliver essential information required by the master node.

        Parameters:
        -----------
        request: str
            Request to the robot to deliver actions
        response: str
            The actions a robot can do, will be populated during execution

        Returns
        -------
        str
            The robot steps it can do
        """
        response.description_response = str(self.description)

        return response

    def actionCallback(self, request, response):
        '''
        The actionCallback function is a service that can be called to execute the available actions the robot
        can preform.
        arameters:
        -----------
        request: str
            Request to the robot to deliver actions
        response: str
            The actions a robot can do, will be populated during execution

        Returns
        -------
        str
            The robot steps it can do
        """
        '''
        self.pf400.force_initialize_robot()

        if request.action_handle == "explore_workcell":

            while self.state != "READY":
                self.get_logger().info("Waiting for PF400 to switch READY state...")
            vars = eval(request.vars)
            print(vars)

            module_list = self.module_explorer.explore_workcell()     #Recieve the module list
            self.get_logger().info(module_list)

        if request.action_handle == "transfer":

            while self.state != "READY":
                print("Waiting for PF400 to switch READY state...")

            source_plate_rotation = ""
            target_plate_rotation = ""
            
            # self.state = "BUSY"
            # self.stateCallback()
            vars = eval(request.vars)
            print(vars)


            if 'source' not in vars.keys():
                print("Pick up location is not provided")
                return 
            elif 'target' not in vars.keys():
                print("Drop off up location is not provided")
                return 

            if len(vars.get('source')) != 6:
                print("Position 1 should be six joint angles lenght")
                return
                
            if len(vars.get('target')) != 6:
                print("Position 2 should be six joint angles lenght")
                return

            if 'source_plate_rotation' not in vars.keys():
                print("Setting source plate rotation to 0")
            else:
                source_plate_rotation = str(vars.get('source_plate_rotation'))

            if 'target_plate_rotation' not in vars.keys():
                print("Setting target plate rotation to 0")
            else:
                target_plate_rotation = str(vars.get('target_plate_rotation'))

            source = vars.get('source')
            print("Source location: ", source)
            target = vars.get('target')
            print("Target location: ",target)

            self.pf400.transfer(source, target, source_plate_rotation, target_plate_rotation)

        elif request.action_handle == "remove_lid":

            while self.state != "READY":
                print("Waiting for PF400 to switch READY state...")

            target_plate_rotation = ""
    
            # self.state = "BUSY"
            # self.stateCallback()
            vars = eval(request.vars)
            print(vars)

            if 'target' not in vars.keys():
                print("Drop off up location is not provided")
                return 
            if len(vars.get('target')) != 6:
                print("Position 2 should be six joint angles lenght")
                return

            if 'target_plate_rotation' not in vars.keys():
                print("Setting target plate rotation to 0")
            else:
                target_plate_rotation = str(vars.get('target_plate_rotation'))
          
            target = vars.get('target')
            print("Target location: ",target)

            if 'lid_hight' not in vars.keys():
                print('Using defult lid hight')
                self.pf400.remove_lid(target, target_plate_rotation)

            else:    
                lid_hight = vars.get('lid_hight')
                print("Lid hight: ",lid_hight)
                self.pf400.remove_lid(target, lid_hight, target_plate_rotation)


        elif request.action_handle == "replace_lid":

            while self.state != "READY":
                print("Waiting for PF400 to switch READY state...")

            target_plate_rotation = ""
    
            # self.state = "BUSY"
            # self.stateCallback()
            vars = eval(request.vars)
            print(vars)

            if 'target' not in vars.keys():
                print("Drop off up location is not provided")
                return 
            if len(vars.get('target')) != 6:
                print("Position 2 should be six joint angles lenght")
                return

            if 'target_plate_rotation' not in vars.keys():
                print("Setting target plate rotation to 0")
            else:
                target_plate_rotation = str(vars.get('target_plate_rotation'))
            
            target = vars.get('target')
            print("Target location: ",target)

            if 'lid_hight' not in vars.keys():
                print('Using defult lid hight')
                self.pf400.remove_lid(target, target_plate_rotation)

            else:    
                lid_hight = vars.get('lid_hight')
                print("Lid hight: ",lid_hight)
                self.pf400.remove_lid(target, lid_hight, target_plate_rotation)
                
        if self.pf400.plate_state == -1:
            self.state = "ERROR"
            self.get_logger().error("Transfer cannot be completed, missing plate!")
        else:
            self.state = "COMPLETED"

        if self.pf400.robot_state == "ERROR":
            self.state = self.pf400.robot_state

        return response

    def whereJCallback(self, request, response):
        '''
        The descriptionCallback function is a service that can be called to showcase the available actions a robot
        can preform as well as deliver essential information required by the master node.
        '''

        self.get_logger().info('What are my joint positions?')
 
        var  = self.pf400.send_command("wherej")
        print(var)
        return response



def main(args = None):

    NAME = "PF400_Client_Node"
    rclpy.init(args=args)  # initialize Ros2 communication
    node = PF400ClientNode(NODE_NAME=NAME)
    rclpy.spin(node)     # keep Ros2 communication open for action node
    rclpy.shutdown()     # kill Ros2 communication

if __name__ == '__main__':
    main()