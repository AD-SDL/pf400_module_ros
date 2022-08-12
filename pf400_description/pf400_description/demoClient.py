import sys
from operator import add

import rclpy
from rclpy.node import Node
from time import sleep

from pf400_module_services.srv import MoveJ 
from pf400_module_services.srv import PeelerActions
from pf400_module_services.srv import SciclopsActions


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


    def workflowDemo(self):
        '''example flow'''


        # sleep(0.5)

        # self.pf400Req.joint_positions = self.pf400_neutral
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)  

        # self.pf400Req.joint_positions = list(map(add, self.cyclops_ext, self.above))
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)  

        self.cyclops2sealer = [262.550, 20.608, 119.290, 662.570, self.gripper_open, 574.367, 264.584, -29.413, 284.376, 372.338, self.gripper_open, 651.621]


        # WIP: Running services across 2 packages

        self.sciclopsReq.action_request = "Get Plate 1"
        self.future = self.cli.call_async(self.sciclopsReq.action_request)
        rclpy.spin_until_future_complete(self, self.future)

        sleep(20)

        self.pf400Req.joint_positions = self.cyclops2sealer
        self.future = self.pf400Client.call_async(self.pf400Req)
        rclpy.spin_until_future_complete(self, self.future)  

        # sleep(1)

        # self.pf400Req.joint_positions = self.cyclopsClosed_ext
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)  

        # self.pf400Req.joint_positions = list(map(add, self.cyclopsClosed_ext, self.above))
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)

        # self.pf400Req.joint_positions = self.pf400_neutralClosed
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)  

        # self.pf400Req.joint_positions = self.module_neutralClosed
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)

        # sleep(0.5)


        # self.pf400Req.joint_positions = list(map(add, self.sealerClosedPos, self.above))
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)

        # self.pf400Req.joint_positions = self.sealerClosedPos
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)

        # sleep(1)

        # self.pf400Req.joint_positions = self.sealerPos
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)


        # self.pf400Req.joint_positions = list(map(add, self.sealerPos, self.above))
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)


        # sleep(20)

        # self.pf400Req.joint_positions = self.sealerPos
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)

        # sleep(1)

        # self.pf400Req.joint_positions = self.sealerClosedPos
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)
        
        # self.pf400Req.joint_positions = list(map(add, self.sealerClosedPos, self.above))
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)


        # self.pf400Req.joint_positions = self.module_neutralClosed
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)

        # self.pf400Req.joint_positions = list(map(add, self.peelerClosedPos, self.above))
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)

        # self.pf400Req.joint_positions = self.peelerClosedPos
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)

        # # sleep(1)

        # self.pf400Req.joint_positions = self.peelerPos
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)

        # sleep(1)

        # self.pf400Req.joint_positions = list(map(add, self.peelerPos, self.above))
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)

        # sleep(20)

        # self.pf400Req.joint_positions = self.peelerPos
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)

        # sleep(1)

        # self.pf400Req.joint_positions = self.peelerClosedPos
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)

        # sleep(1)

        # self.pf400Req.joint_positions = list(map(add, self.peelerClosedPos, self.above))
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)

        # self.pf400Req.joint_positions = self.module_neutralClosed
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)

        # self.pf400Req.joint_positions = self.pf400_neutralClosed
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)  

        # self.pf400Req.joint_positions = list(map(add, self.cyclopsClosed_ext, self.above))
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)

        # self.pf400Req.joint_positions = self.cyclopsClosed_ext
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)  

        # self.pf400Req.joint_positions = self.cyclops_ext
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)  

        # sleep(0.5)


        # self.pf400Req.joint_positions = list(map(add, self.cyclops_ext, self.above))
        # self.future = self.cli.call_async(self.pf400Req)
        # rclpy.spin_until_future_complete(self, self.future)

        # self.pf400Req.joint_positions = self.pf400_neutral
        # self.future = self.cli.call_async(self.pf400Req)
        rclpy.spin_until_future_complete(self, self.future)  



        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.workflowDemo()

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()