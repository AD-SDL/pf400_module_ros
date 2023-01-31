import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# from rclpy.clock import clock

from threading import Thread
from time import sleep

from std_msgs.msg import String
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from pf400_driver.pf400_driver import PF400

class PF400DescriptionClient(Node):

    def __init__(self, NODE_NAME = 'PF400DescriptionNode'):
        super().__init__(NODE_NAME)

        timer_period = 0.1  # seconds
        self.declare_parameter("ip","127.0.0.1")
        self.declare_parameter("port",8085)

        # Receiving the real IP and PORT from the launch parameters
        self.ip =  self.get_parameter("ip").get_parameter_value().string_value
        self.port = self.get_parameter("port").get_parameter_value().integer_value

        self.get_logger().info("Received IP: " + self.ip + " Port:" + str(self.port))
        self.state = "UNKNOWN"
        self.connect_robot()

        joint_cb_group = ReentrantCallbackGroup()
        state_cb_group = ReentrantCallbackGroup()

        self.statePub = self.create_publisher(String, NODE_NAME + '/state',10)
        self.stateTimer = self.create_timer(timer_period, callback = self.stateCallback, callback_group = state_cb_group)

        self.joint_publisher = self.create_publisher(JointState,'joint_states', 10, callback_group = joint_cb_group)
        self.joint_state_handler = self.create_timer(timer_period, callback = self.joint_state_publisher_callback, callback_group = joint_cb_group)
   
    def connect_robot(self):
        try:
            self.pf400 = PF400(self.ip, self.port)

        except Exception as error_msg:
            self.state = "PF400 CONNECTION ERROR"
            self.get_logger().error("------- PF400 Error message: " + str(error_msg) +  (" -------"))

        else:
            self.get_logger().info("PF400 online")
    
    def stateCallback(self):
        '''
        Publishes the pf400_description state to the 'state' topic. 
        '''
        if self.state != "PF400 CONNECTION ERROR":
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
            self.get_logger().info(msg.data)
            sleep(0.5)

        else: 
            msg = String()
            msg.data = 'State: %s' % self.state
            self.statePub.publish(msg)
            self.get_logger().error(msg.data)
            self.get_logger().warn("Trying to connect again! IP: " + self.ip + " Port:" + str(self.port))
            self.connect_robot()


    def joint_state_publisher_callback(self):
        
        if self.state == "PF400 CONNECTION ERROR":
            return

        joint_states = self.pf400.refresh_joint_state()
        pf400_joint_msg = JointState()
        pf400_joint_msg.header = Header()
        pf400_joint_msg.header.stamp = self.get_clock().now().to_msg()
        pf400_joint_msg.name = ['J1', 'J2', 'J3', 'J4', 'J5','J5_mirror', 'J6']
        pf400_joint_msg.position = joint_states
        pf400_joint_msg.velocity = []
        pf400_joint_msg.effort = []
        self.joint_publisher.publish(pf400_joint_msg)
        self.get_logger().info('Publishing joint states: "%s"' % joint_states)


def main(args=None):
    rclpy.init(args=args)
    try:
        pf400_joint_state_publisher = PF400DescriptionClient()
        executor = MultiThreadedExecutor()
        executor.add_node(pf400_joint_state_publisher)

        try:
            pf400_joint_state_publisher.get_logger().info('Beginning client, shut down with CTRL-C')
            executor.spin()
        except KeyboardInterrupt:
            pf400_joint_state_publisher.get_logger().info('Keyboard interrupt, shutting down.\n')
        finally:
            executor.shutdown()
            pf400_joint_state_publisher.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()