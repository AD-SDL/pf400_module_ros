import sys

import rclpy
from rclpy.node import Node

from pf400_module_services.srv import MoveJ 


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')


        self.module_neutral = [400, 0, 237.92, 388.47, 76.7, 643.54]

        self.peeler_above_open = [307.419, -31.460, 289.067, 373.143, 90.0, 661.437]
        self.peeler_above_closed = [307.419, -31.460, 289.067, 373.143, 76.7, 661.437]

        self.peeler_open = [272.419, -31.460, 289.067, 373.143, 90.0, 661.437]
        self.peeler_closed = [272.419, -31.460, 289.067, 373.143, 76.7, 661.437]

        self.sealer_above_closed = [250.966, -32.123, 304.570, 356.268, 76.7, 812.485]
        self.sealer_above_open = [250.966, -32.123, 304.570, 356.268, 90.0, 812.485]

        self.sealer_closed = [220.960, -30.670, 302.227, 356.440, 76.7, 812.442]
        self.sealer_open = [220.960, -30.670, 302.227, 356.440, 90.0, 812.442]

        self.module_switch = [260.241, -10.147, 174.616, 539.662, 76.7, -254.6]
        self.ot2_enter = [260.241, -10.465, 60.89, 537.196, 76.7, -254.578]

        self.ot2_above_closed = [231.183, 29.345, 89.351, 598.682, 76.7, -67.409]
        self.ot2_above_open = [231.183, 29.345, 89.351, 598.682, 90.0, -67.409]

        self.ot2_closed = [201.183, 29.345, 89.351, 598.682, 76.7, -67.409]
        self.ot2_open = [201.183, 29.345, 89.351, 598.682, 90.0, -67.409]

        self.cli = self.create_client(MoveJ, 'pf400_moveJ')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.pf400Req = MoveJ.Request()

    def pf400ToPeeler(self):
        self.pf400Req.joint_positions = self.peeler_above_open
        self.future = self.cli.call_async(self.pf400Req)
        rclpy.spin_until_future_complete(self, self.future)
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