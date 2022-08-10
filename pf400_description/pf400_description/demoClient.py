import sys

import rclpy
from rclpy.node import Node
from time import sleep

from pf400_module_services.srv import MoveJ 
from pf400_module_services.srv import PeelerActions


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')

        self.module_neutral = [400.0, 0.1, 237.92, 388.47, 76.7, 643.54]


        self.gripper_open = 90.0
        self.gripper_closed = 76.7

        self.pos1 = [272.419, -31.460, 289.067, 373.143, self.gripper_open, 661.437]
        self.pos2 = [272.419, -31.460, 289.067, 373.143, self.gripper_open, 661.437]

        self.above = [35.0, 0.0, 0.0, 0.0, 0.0, 0.0]


        # self.peeler_above_open = self.pos1 + [35.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # self.peeler_above_closed = self.pos1 + [35.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # self.peeler_open = [272.419, -31.460, 289.067, 373.143, 90.0, 661.437]
        # self.peeler_closed = [272.419, -31.460, 289.067, 373.143, 76.7, 661.437]

        # self.sealer_above_closed = [250.966, -32.123, 304.570, 356.268, 76.7, 812.485]
        # self.sealer_above_open = [250.966, -32.123, 304.570, 356.268, 90.0, 812.485]

        # self.sealer_closed = [220.960, -30.670, 302.227, 356.440, 76.7, 812.442]
        # self.sealer_open = [220.960, -30.670, 302.227, 356.440, 90.0, 812.442]

        # self.module_switch = [260.241, -10.147, 174.616, 539.662, 76.7, -254.6]
        # self.ot2_enter = [260.241, -10.465, 60.89, 537.196, 76.7, -254.578]

        # self.ot2_above_closed = [231.183, 29.345, 89.351, 598.682, 76.7, -67.409]
        # self.ot2_above_open = [231.183, 29.345, 89.351, 598.682, 90.0, -67.409]

        # self.ot2_closed = [201.183, 29.345, 89.351, 598.682, 76.7, -67.409]
        # self.ot2_open = [201.183, 29.345, 89.351, 598.682, 90.0, -67.409]

        self.cli = self.create_client(MoveJ, 'pf400_moveJ')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.pf400Req = MoveJ.Request()


    # def grab_plate(self, pos, neutral_pos=None):
    #     if neutral_pos:
    #         pass
        
    #     #move to transfer_pos (arm parallel to rail)
    #     #move to neutral
    #     #move to pos + z
    #     #move to pos + z + gripper_ope
    #     #move to pos
    #     #move to pos + gripper_close
    #     #move to pos + z + gripper_close
    #     #move to neutral( gripper close)

    # def deliver_plate(self, pos, neutral_pos)

    #     #move to tranfer_pos
    #     #move to neutral
    #     #move to pos + z
    #     #move to pos + 5
    #     #open
    #     #move to pos + z (open)
    #     #go back

    def transfer(self, pos1, pos2, neutral1, neutral2):
        self.grab_plate(pos1, neutral1)
        self.deliver_plate(pos2, neutral2)

    def pf400ToPeeler(self):

        self.pf400Req.joint_positions = self.module_neutral
        self.future = self.cli.call_async(self.pf400Req)
        rclpy.spin_until_future_complete(self, self.future)

        sleep(0.5)

        self.pf400Req.joint_positions = self.pos1 + self.above
        self.future = self.cli.call_async(self.pf400Req)
        rclpy.spin_until_future_complete(self, self.future)

        self.pf400Req.joint_positions = self.module_neutral
        self.future = self.cli.call_async(self.pf400Req)
        rclpy.spin_until_future_complete(self, self.future)

        sleep(0.5)

        return self.future.result()

    def pf400ToSealer(self, a, b):

        return 0

    def pf400ToOt2(self, a, b):

        return 0

    def peelerRequest(self, a, b):

        return 0


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.pf400ToPeeler()

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()