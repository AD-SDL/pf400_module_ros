#! /usr/bin/env python3

import rclpy                 # import Rospy
from rclpy.node import Node  # import Rospy Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from std_msgs.msg import String
from std_srvs.srv import Empty

from time import sleep

from threading import Thread

from wei_services.srv import WeiDescription 
from wei_services.srv import WeiActions  

from pf400_driver.pf400_driver import PF400
from pf400_driver.pf400_camera_driver import PF400_CAMERA

class PF400Client(Node):
    '''
    The jointControlNode inputs data from the 'action' topic, providing a set of commands for the driver to execute. It then receives feedback, 
    based on the executed command and publishes the state of the peeler and a description of the peeler to the respective topics.
    '''
    def __init__(self, TEMP_NODE_NAME = "PF400_Client_Node"):
        '''
        The init function is neccesary for the peelerNode class to initialize all variables, parameters, and other functions.
        Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
        '''

        super().__init__(TEMP_NODE_NAME)
        node_name = self.get_name()

        # Setting temporary default parameter values        
        self.declare_parameter("ip","127.0.0.1")
        self.declare_parameter("port",8085)

        # Receiving the real IP and PORT from the launch parameters
        self.ip =  self.get_parameter("ip").get_parameter_value().string_value
        self.port = self.get_parameter("port").get_parameter_value().integer_value

        self.get_logger().info("Received IP: " + self.ip + " Port:" + str(self.port))
        self.state = "UNKNOWN"
        self.job_flag = False
        self.movement_state = -1
        self.past_movement_state = -1
        self.state_refresher_timer = 0

        self.connect_robot()

        action_cb_group = ReentrantCallbackGroup()
        description_cb_group = ReentrantCallbackGroup()
        state_cb_group = ReentrantCallbackGroup()
        state_refresher_cb_group = ReentrantCallbackGroup()

        timer_period = 0.5  # seconds

        self.statePub = self.create_publisher(String, node_name + '/state', 10)
        self.stateTimer = self.create_timer(timer_period, callback = self.stateCallback, callback_group = state_cb_group)
        
        self.StateRefresherTimer = self.create_timer(timer_period, callback = self.robot_state_refresher_callback, callback_group = state_refresher_cb_group)
        # state_thread = Thread(target = self.stateCallback)
        # state_thread.start()

        self.action_handler = self.create_service(WeiActions, node_name + "/action_handler", self.actionCallback, callback_group = action_cb_group)
        self.description_handler = self.create_service(WeiDescription, node_name + "/description_handler", self.descriptionCallback, callback_group = description_cb_group)

        self.description={}

    def connect_robot(self):
        try:
            self.pf400 = PF400(self.ip, self.port)

        except Exception as error_msg:
            self.state = "PF400 CONNECTION ERROR"
            self.get_logger().error("------- PF400 Error message: " + str(error_msg) +  (" -------"))

        else:
            self.get_logger().info("PF400 online")
            self.pf400.initialize_robot()
            self.module_explorer = PF400_CAMERA(self.pf400)

    def robot_state_refresher_callback(self):
        "Refreshes the robot states if robot cannot update the state parameters automatically because it is not running any jobs"
        try:
            if self.job_flag == False: #Only refresh the state manualy if robot is not running a job.
                self.pf400.get_robot_movement_state()
                self.pf400.get_overall_state()
                self.get_logger().info("Refresh state")
                self.state_refresher_timer = 0 
            
            if self.past_movement_state == self.movement_state:
                self.state_refresher_timer += 1
            elif self.past_movement_state != self.movement_state:
                self.past_movement_state = self.movement_state
                self.state_refresher_timer = 0 

            if self.state_refresher_timer > 25: # Refresh the state if robot has been stuck at a status for more than 25 refresh times.
                self.pf400.get_robot_movement_state()
                self.pf400.get_overall_state()
                self.get_logger().info("Refresh state")
                self.job_flag = False

        except Exception as err:
            # self.state = "PF400 CONNECTION ERROR"
            self.get_logger().error(str(err))

    def stateCallback(self):
        '''
        Publishes the pf400 state to the 'state' topic. 
        '''
        msg = String()

        try:
            self.movement_state = self.pf400.movement_state
            self.get_logger().warn("Move state: " + str(self.movement_state))

        except Exception as err:
            self.get_logger().error("ROBOT IS NOT RESPONDING! ERROR: " + str(err))
            self.state = "PF400 CONNECTION ERROR"

        if self.state != "PF400 CONNECTION ERROR":

            if self.pf400.attach_state == "-1":
                msg.data = "State: Robot is not attached"
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)
                self.pf400.attach_robot()
                sleep(6) 

            if self.movement_state == 0:
                self.state = "POWER OFF"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().warn(msg.data)
                self.pf400.force_initialize_robot()
                self.job_flag = False

            elif self.movement_state == 1 and self.job_flag == False:
                self.state = "READY"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)

            if self.state == "COMPLETED":
                self.job_flag = False
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)

            elif self.pf400.robot_state == "ERROR":
                self.state = "ERROR"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().error(msg.data)
                self.get_logger().error("Error Message: " + self.pf400.robot_error_msg)
                self.job_flag = False

            elif (self.movement_state >= 1 and self.job_flag == True) or  self.movement_state >= 2:
                self.state = "BUSY"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)
            # else: 
            #     self.state = "ERROR"
            #     msg.data = 'State: %s' % self.state
            #     self.statePub.publish(msg)
            #     self.get_logger().error("DATA LOSS")
            #     # self.job_flag = False
        else: 
            msg.data = 'State: %s' % self.state
            self.statePub.publish(msg)
            self.get_logger().error(msg.data)
            self.get_logger().warn("Trying to connect again! IP: " + self.ip + " Port:" + str(self.port))
            self.connect_robot()

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
        request.action_handle: str
            Request to the robot to deliver actions
        request.vars: str
            Request to the robot to deliver actions
        response.action_response: int16
            Request to the robot to deliver actions
        response.action_msg: str
        Returns
        -------
        str
            The robot steps it can do
        """
        '''

        if self.state == "PF400 CONNECTION ERROR":
            self.get_logger().error("Connection error, cannot accept a job!")
            return

        while self.state != "READY":
            self.get_logger().warn("Waiting for PF400 to switch READY state...")
            sleep(0.5)

        self.job_flag = True    
        # self.pf400.force_initialize_robot()
        self.get_logger().info('Received Action: ' + request.action_handle)

        err=0

        if request.action_handle == "explore_workcell":
            
            vars = eval(request.vars)
            self.get_logger().info(vars)

            module_list = self.module_explorer.explore_workcell()     #Recieve the module list
            self.get_logger().info(str(module_list))
            if module_list:
                action_response = 0
            response.action_response = action_response
            response.action_msg= str(module_list)
            self.get_logger().info('Finished Action: ' + request.action_handle)
            self.state = "COMPLETED"
            return response

        if request.action_handle == "transfer":

            source_plate_rotation = ""
            target_plate_rotation = ""
            
            vars = eval(request.vars)
            self.get_logger().info(str(vars))

            if 'source' not in vars.keys():
                err = 1
                msg = "Pick up location is not provided. Canceling the job!"
            elif 'target' not in vars.keys():
                err = 1
                msg = "Drop off up location is not provided. Canceling the job!"
            elif len(vars.get('source')) != 6:
                err = 1
                msg = "Position 1 should be six joint angles lenght. Canceling the job!"
            elif len(vars.get('target')) != 6:
                err = 1
                msg = "Position 2 should be six joint angles lenght. Canceling the job!"

            if err:
                response.action_response = -1
                response.action_msg= msg
                self.get_logger().error('Error: ' + msg)
                self.state = "COMPLETED"
                return response

            if 'source_plate_rotation' not in vars.keys():
                self.get_logger().info("Setting source plate rotation to 0")
            else:
                source_plate_rotation = str(vars.get('source_plate_rotation'))

            if 'target_plate_rotation' not in vars.keys():
                self.get_logger().info("Setting target plate rotation to 0")
            else:
                target_plate_rotation = str(vars.get('target_plate_rotation'))

            source = vars.get('source')
            self.get_logger().info("Source location: " + str(source))
            target = vars.get('target')
            self.get_logger().info("Target location: "+ str(target))
            
            self.pf400.transfer(source, target, source_plate_rotation, target_plate_rotation)

            response.action_response = 0
            response.action_msg= "All good PF400"
            self.get_logger().info('Finished Action: ' + request.action_handle)
            self.state = "COMPLETED"
            return response

        if request.action_handle == "remove_lid":

            target_plate_rotation = ""

            vars = eval(request.vars)
            self.get_logger().info(str(vars))

            if 'target' not in vars.keys():
                self.get_logger().error("Drop off up location is not provided. Canceling the job!")
                self.state = "COMPLETED"
                return 

            if len(vars.get('target')) != 6:
                self.get_logger().error("Position 2 should be six joint angles lenght. Canceling the job!")
                self.state = "COMPLETED"
                return

            if 'target_plate_rotation' not in vars.keys():
                self.get_logger().info("Setting target plate rotation to 0")
            else:
                target_plate_rotation = str(vars.get('target_plate_rotation'))
          
            target = vars.get('target')
            self.get_logger().info("Target location: " + str(target))

            lid_height = vars.get('lid_height', 7.0)
            self.get_logger().info("Lid hight: " + str(lid_height))
                

            self.pf400.remove_lid(target, lid_height, target_plate_rotation)
            response.action_response = 0
            response.action_msg= "All good PF400"
            self.state = "COMPLETED"

            return response

        if request.action_handle == "replace_lid":

            target_plate_rotation = ""
    
            # self.state = "BUSY"
            # self.stateCallback()
            vars = eval(request.vars)
            self.get_logger().info(vars)

            if 'target' not in vars.keys():
                self.get_logger().error("Drop off location is not provided. Canceling the job!")
                self.state = "COMPLETED"
                return 

            if len(vars.get('target')) != 6:
                self.get_logger().error("Position 2 should be six joint angles lenght. Canceling the job!")
                self.state = "COMPLETED"
                return

            if 'target_plate_rotation' not in vars.keys():
                self.get_logger().info("Setting target plate rotation to 0")
            else:
                target_plate_rotation = str(vars.get('target_plate_rotation'))
            
            target = vars.get('target')
            self.get_logger().info("Target location: ",target)

            if 'lid_height' not in vars.keys():
                self.get_logger().info('Using defult lid hight')
                self.pf400.remove_lid(target, target_plate_rotation)

            else:    
                lid_height = vars.get('lid_height')

                self.get_logger().info("Lid hight: " + str(lid_height))
                self.pf400.remove_lid(target, lid_height, target_plate_rotation)
                
        if self.pf400.plate_state == -1:
            self.state = "ERROR"
            self.get_logger().error("Transfer cannot be completed, missing plate!")
        else:
            self.state = "COMPLETED"

        # if self.pf400.robot_state == "ERROR":
        #     self.state = self.pf400.robot_state

        #TODO: move every action into its own return
        self.state = "COMPLETED"
        return response

    def whereJCallback(self, request, response):
        '''
        The descriptionCallback function is a service that can be called to showcase the available actions a robot
        can preform as well as deliver essential information required by the master node.
        '''

        self.get_logger().info('What are my joint positions?')
 
        var  = self.pf400.send_command("wherej")
        self.get_logger().info(str(var))
        return response

def main(args = None):

    rclpy.init(args=args)  # initialize Ros2 communication

    try:
        pf400_client = PF400Client()
        executor = MultiThreadedExecutor()
        executor.add_node(pf400_client)

        try:
            pf400_client.get_logger().info('Beginning client, shut down with CTRL-C')
            executor.spin()
        except KeyboardInterrupt:
            pf400_client.get_logger().info('Keyboard interrupt, shutting down.\n')
        finally:
            executor.shutdown()
            pf400_client.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()