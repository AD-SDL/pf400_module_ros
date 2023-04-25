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

from pf400_driver.errors import ConnectionException, CommandException
from pf400_driver.pf400_driver import PF400
# from pf400_driver.errors import ConnectionException, CommandException
from pf400_driver.pf400_camera_driver import PF400_CAMERA

class PF400Client(Node):
    '''
    The PF400Client inputs data from the 'action' topic, providing a set of commands for the driver to execute. It then receives feedback, 
    based on the executed command and publishes the state of the PF400 and a description of the PF400 to the respective topics.
    '''
    def __init__(self, TEMP_NODE_NAME = "PF400_Client_Node"):
        """ Connect to the robot by calling the PF400 object from the pf400_driver
       
         Parameters:
        -----------
            str
                TEMP_NODE_NAME: A temporary node name to create the intial ROS node. Actual Node is later recieved by the launch parameters.
            
        Returns
        -------
            None
        """

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
        self.pf400_error_message = ""
        self.pf400_state = ""
        self.action_flag = "READY"
        self.movement_state = -1
        self.past_movement_state = -1
        self.state_refresher_timer = 0

        self.connect_robot()
        sleep(1) # Sleep till robot connection is established to start checking for state information 
        self.stateRefresherCallback() 

        action_cb_group = ReentrantCallbackGroup()
        description_cb_group = ReentrantCallbackGroup()
        state_cb_group = ReentrantCallbackGroup()
        state_refresher_cb_group = ReentrantCallbackGroup()

        state_publisher_period = 0.5  # seconds
        self.state_refresher_period = state_publisher_period + 1.0  # seconds

        self.statePub = self.create_publisher(String, node_name + '/state', 10)
        self.stateTimer = self.create_timer(state_publisher_period, callback = self.stateCallback, callback_group = state_cb_group)
        
        self.StateRefresherTimer = self.create_timer(self.state_refresher_period, callback = self.stateRefresherCallback, callback_group = state_refresher_cb_group)

        self.action_handler = self.create_service(WeiActions, node_name + "/action_handler", self.actionCallback, callback_group = action_cb_group)
        self.description_handler = self.create_service(WeiDescription, node_name + "/description_handler", self.descriptionCallback, callback_group = description_cb_group)

        self.description={}

    def connect_robot(self):
        """ Connect to the robot by calling the PF400 object from the pf400_driver
       
         Parameters:
        -----------
            None

        Returns
        -------
            None
        """
        
        try:
            self.pf400 = PF400(self.ip, self.port)
            self.pf400.initialize_robot()
            self.module_explorer = PF400_CAMERA(self.pf400)

        except ConnectionException as error_msg:
            self.state = "PF400 CONNECTION ERROR"
            self.get_logger().error(str(error_msg))

        except Exception as err:
            self.state = "PF400 ERROR"
            self.get_logger().error(str(err))
            
        else:
            self.get_logger().info("PF400 online")


    def stateRefresherCallback(self):
        """ Refreshes the robot states if robot cannot update the state parameters automatically because it is not running any jobs
       
         Parameters:
        -----------
            None
        Returns
        -------
            None
        """
        err = None
        try:

            if self.action_flag == "READY": #Only refresh the state manualy if robot is not running a job.
                self.pf400.get_robot_movement_state()
                self.pf400.get_overall_state()
                # self.get_logger().info("Refresh state")
                self.state_refresher_timer = 0 

            elif self.state_refresher_timer > 60: # Refresh the state if robot has been stuck at the same status for more than 60x1.5 seconds.
                # self.state_refresher_counter += 0
                # if self.state_refresher_counter > 3: # Before calling the state refresher functions make sure this condition is consistant for at least 3 cycles.
                self.pf400.get_robot_movement_state()
                self.pf400.get_overall_state()
                self.get_logger().info("Refresh state, robot state is frozen...")
                self.action_flag = "READY"
                self.state_refresher_counter = 0

            if self.past_movement_state == self.movement_state:
                self.state_refresher_timer += 1
            elif self.past_movement_state != self.movement_state:
                self.past_movement_state = self.movement_state
                self.state_refresher_timer = 0 

        except ConnectionException as connection_err:
            err = connection_err

        except CommandException as command_err:
            err = command_err

        except UnboundLocalError as local_var_err:
            err = local_var_err

        except TimeoutError as time_err:
            err = time_err

        except AttributeError as attribute_err:
            err = attribute_err
            self.get_logger().warn("Trying to connect again! IP: " + self.ip + " Port:" + str(self.port))
            self.connect_robot()

        finally:
            if err:
                self.state = "ERROR"
                self.get_logger().error(str(err))
         

    def stateCallback(self):
        """ Publishes the pf400 state to the 'state' topic. 

        Parameters:
        -----------
            None
        Returns
        -------
            None
        """
        msg = String()
        try_connect = False
        err = None
        err_flag = False
        
        try:
            self.movement_state = self.pf400.movement_state
            # self.get_logger().warn("Move state: " + str(self.movement_state))

        except UnboundLocalError as local_var_err:
            err = local_var_err

        except AttributeError as attribute_err:
            err = attribute_err
            try_connect = True

        except Exception as general_err:
            err = general_err

        finally:
            if try_connect:
                self.state = "ERROR"
                self.get_logger().error(str(err))
                self.get_logger().warn("Trying to connect again! IP: " + self.ip + " Port:" + str(self.port))
                self.connect_robot()

            if err:
                self.state = "ERROR"
                self.get_logger().error(str(err))
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().error(msg.data)
                return
            
        # Check if robot wasn't attached to the software after recovering from Power Off state
        if self.pf400.attach_state == "-1":
            self.state = "ERROR"
            self.get_logger().warn("Robot is not attached")
            err_flag = True
            self.pf400.force_initialize_robot()

        # Publishing robot warning messages if the job wasn't completed successfully
        if self.pf400.robot_warning.upper() != "CLEAR" and len(self.pf400.robot_warning)>0:
            self.state = "ERROR"
            self.get_logger().warn(self.pf400.robot_warning)
            err_flag = True
            self.pf400.robot_warning = "CLEAR"
            self.action_flag = "READY"

        # Checking real robot state parameters and publishing the current state
        if self.movement_state == 0:
            self.state = "POWER OFF"
            err_flag = True
            self.pf400.force_initialize_robot()
            self.action_flag = "READY"

        elif self.pf400.robot_state == "ERROR" or self.state == "ERROR":
            self.state = "ERROR"
            err_flag = True
            self.get_logger().error(self.pf400.robot_error_msg)
            self.action_flag = "READY"
            self.state = "UNKOWN"

        elif self.state == "COMPLETED" and self.action_flag == "BUSY":
            self.action_flag = "READY"

        elif (self.movement_state >= 1 and self.action_flag == "BUSY") or self.movement_state >= 2:
            self.state = "BUSY"

        elif self.movement_state == 1 and self.action_flag == "READY":
            self.state = "READY"

        msg.data = 'State: %s' % self.state
        self.statePub.publish(msg)

        if err_flag:
            self.get_logger().error(msg.data)
        else:
            self.get_logger().info(msg.data)

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
            message = "Connection error, cannot accept a job!"
            self.get_logger().error(message)
            response.action_response = -1
            response.action_msg= message
            return response

        while self.state != "READY":
            self.get_logger().warn("Waiting for PF400 to switch READY state...")
            sleep(0.2)

        self.action_flag = "BUSY"    
        self.get_logger().info('Received Action: ' + request.action_handle.upper())
        sleep(self.state_refresher_period + 0.1) #Before starting the action, wait for stateRefresherCallback function to cycle for at least once to avoid data loss.

        vars = eval(request.vars)
        self.get_logger().info(str(vars))

        err=False

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

        elif request.action_handle == "transfer":

            source_plate_rotation = ""
            target_plate_rotation = ""

            if 'source' not in vars.keys():
                err = True
                msg = "Pick up location is not provided. Canceling the job!"
            elif 'target' not in vars.keys():
                err = True
                msg = "Drop off up location is not provided. Canceling the job!"
            elif len(vars.get('source')) != 6:
                err = True
                msg = "Position 1 should be six joint angles lenght. Canceling the job!"
            elif len(vars.get('target')) != 6:
                err = True
                msg = "Position 2 should be six joint angles lenght. Canceling the job!"

            if err:
                response.action_response = -1
                response.action_msg= msg
                self.get_logger().error('Error: ' + msg)
                self.state = "ERROR"
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
            
            try:
                self.pf400.transfer(source, target, source_plate_rotation, target_plate_rotation)

            except Exception as err:
                response.action_msg = "Transfer failed. Error:" + err
                response.action_response = -1
                if self.pf400.robot_warning.upper() != "CLEAR":
                    response.action_msg = self.pf400.robot_warning.upper()
                self.state = "ERROR"

            else:    
                response.action_response = 0
                response.action_msg = "PF400 succsessfully completed a transfer"
                self.state = "COMPLETED"

            finally:
                self.get_logger().info('Finished Action: ' + request.action_handle)
                return response

        elif request.action_handle == "remove_lid":

            target_plate_rotation = ""

            if 'target' not in vars.keys():
                err = 1
                msg = "Target location is not provided. Canceling the job!"
                self.get_logger().error(msg)
                self.state = "ERROR"
                 

            if len(vars.get('target')) != 6:
                err = 1
                msg = "Target position should be six joint angles lenght. Canceling the job!"
                self.get_logger().error(msg)
                self.state = "ERROR"
            
            if err:
                response.action_response = -1
                response.action_msg= msg
                self.get_logger().error('Error: ' + msg)
                return response

            if 'target_plate_rotation' not in vars.keys():
                self.get_logger().info("Setting target plate rotation to 0")
            else:
                target_plate_rotation = str(vars.get('target_plate_rotation'))
          
            target = vars.get('target')
            self.get_logger().info("Target location: " + str(target))

            lid_height = vars.get('lid_height', 7.0)
            self.get_logger().info("Lid hight: " + str(lid_height))
                
            try:
                self.pf400.remove_lid(target, lid_height, target_plate_rotation)
            except Exception as err:
                response.action_response = -1
                response.action_msg= "Remove lid failed. Error:" + err
                self.state = "ERROR"
            else:    
                response.action_response = 0
                response.action_msg= "Remove lid successfully completed"
                self.state = "COMPLETED"

            finally:
                self.get_logger().info('Finished Action: ' + request.action_handle)
                return response
            
        elif request.action_handle == "replace_lid":

            target_plate_rotation = ""

            if 'target' not in vars.keys():
                err = 1
                msg = "Target location is not provided. Canceling the job!"
                self.get_logger().error(msg)
                self.state = "ERROR"
                 

            if len(vars.get('target')) != 6:
                err = 1
                msg = "Target position should be six joint angles lenght. Canceling the job!"
                self.get_logger().error(msg)
                self.state = "ERROR"

            if err:
                response.action_response = -1
                response.action_msg= msg
                self.get_logger().error('Error: ' + msg)
                return response
        
            if 'target_plate_rotation' not in vars.keys():
                self.get_logger().info("Setting target plate rotation to 0")
            else:
                target_plate_rotation = str(vars.get('target_plate_rotation'))
            

            if 'lid_height' not in vars.keys():
                self.get_logger().info('Using defult lid hight')
                lid_height = 7.0

            else:    
                lid_height = vars.get('lid_height')

            self.get_logger().info("Lid hight: " + str(lid_height))

            try:    
                self.pf400.replace_lid(target, lid_height, target_plate_rotation)
            except Exception as err:
                response.action_response = -1
                response.action_msg= "Replace lid failed. Error:" + err
                self.state = "ERROR"
            else:    
                response.action_response = 0
                response.action_msg= "Replace lid successfully completed"
                self.state = "COMPLETED"
            finally:
                self.get_logger().info('Finished Action: ' + request.action_handle)
                return response

        else:
            msg = "UNKOWN ACTION REQUEST! Available actions: explore_workcell, transfer, remove_lid, replace_lid"
            response.action_response = -1
            response.action_msg = msg
            self.get_logger().error('Error: ' + msg)
            self.state = "ERROR"
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