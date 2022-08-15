import sys
from operator import add

import rclpy
from rclpy.node import Node
from time import sleep

from std_msgs.msg import String


from pf400_module_services.srv import MoveJ 
from pf400_module_services.srv import PeelerActions
# from pf400_module_services.srv import SciclopsActions
from sciclops_module_services.srv import SciclopsActions


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')


        # self.module_neutral = [400.0, 0.1, 237.92, 388.47, self.gripper_open, 643.54]
        # self.module_neutralClosed = [400.0, 0.1, 237.92, 388.47, self.gripper_closed, 643.54]

        self.peelerPos = [264.584, -29.413,	284.376, 372.338, 0.0, 651.621]
        self.sealerPos = [231.788, -27.154, 313.011, 342.317, 0.0, 683.702]
        self.cyclops_ext = [262.550, 20.608, 119.290, 662.570, 0.0, 574.367]


        self.pf400Client = self.create_client(MoveJ, 'pf400_moveJ')
        while not self.pf400Client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pf400 service not available, waiting again...')
        self.pf400Req = MoveJ.Request()
                                                                

        self.sciclopsClient = self.create_client(SciclopsActions, 'sciclops_actions') # Creating client instance 
        while not self.sciclopsClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('syclops service not available, waiting again...')
        self.sciclopsReq = SciclopsActions.Request()

        self.pf400StateSub = self.create_subscription(String, 'pf400_state', self.pf400StateCallback, 10)

        self.pf400StateSub # prevent unused variable warning

        self.sciclopsStateSub = self.create_subscription(String, 'sciclops_state', self.sciclopsStateCallback, 10)

        self.sciclopsStateSub # prevent unused variable warning

        self.workflowDemo()

    def pf400StateCallback(self, msg):

        self.pf400State = msg.data


    def sciclopsStateCallback(self, msg):

        self.sciclopsState = msg.data



    def workflowDemo(self):
        '''example flow'''

        self.cyclops2sealer = [262.550, 20.608, 119.290, 662.570, 0.0, 574.367, 231.788, -27.154, 313.011, 342.317, 0.0, 683.702]
        self.sealer2peeler = [231.788, -27.154, 313.011, 342.317, 0.0, 683.702, 264.584, -29.413,	284.376, 372.338, 0.0, 651.621]


        self.sciclopsReq.action_request = "Get Plate 1"
        self.future = self.sciclopsClient.call_async(self.sciclopsReq)
        rclpy.spin_until_future_complete(self, self.future)  

        sleep(35)

        self.pf400Req.joint_positions = self.cyclops2sealer
        self.future = self.pf400Client.call_async(self.pf400Req)
        rclpy.spin_until_future_complete(self, self.future)  

        self.pf400Req.joint_positions = self.sealer2peeler
        self.future = self.pf400Client.call_async(self.pf400Req)
        rclpy.spin_until_future_complete(self, self.future)  
        

        rclpy.spin_until_future_complete(self, self.future)  


        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    # rclpy.spin(minimal_client)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()