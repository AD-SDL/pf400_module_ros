import rclpy
from rclpy.node import Node, MutuallyExclusiveCallbackGroup
# from rclpy.clock import clock

from threading import Thread

from std_msgs.msg import String
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from pf400_driver.pf400_driver import PF400

class PF400DescriptionClient(Node):

    def __init__(self, NODE_NAME = 'PF400DescriptionNode'):
        super().__init__(NODE_NAME)

        self.pf400 = PF400("192.168.50.50", "10100")
        self.pf400.initialize_robot()

        timer_period = 0.5  # seconds

        self.state = "UNKNOWN"
        my_callback_group = MutuallyExclusiveCallbackGroup()

        self.joint_publisher = self.create_publisher(JointState,'joint_states',10)
        self.joint_state_handler = self.create_timer(timer_period, callback = self.joint_state_publisher_callback,qos_profile =1,  callback_group= my_callback_group)

        self.statePub = self.create_publisher(String, NODE_NAME + '/state', 10)
        self.stateTimer = self.create_timer(timer_period, callback = self.stateCallback, callback_group=my_callback_group)
        
        # self.joint_publisher = self.create_publisher(JointState,'joint_states',10)
        # self.joint_state_handler = self.create_timer(timer_period, self.joint_state_publisher_callback)

        # self.statePub = self.create_publisher(String, NODE_NAME + '/state', 10)
        # self.stateTimer = self.create_timer(timer_period, self.stateCallback)
        
        # stateTimer = Thread(target=self.stateCallback)
        # joint_state_thread = Thread(target=self.joint_state_publisher_callback)
        # joint_state_thread.start()
        # stateTimer.start()

    
    def stateCallback(self):
        '''
        Publishes the pf400_description state to the 'state' topic. 
        '''
        msg = String()
        msg.data = 'State: %s' % self.state
        self.statePub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.state = "READY"
        # robot.transfer(thermo2, pos1, "wide", "narrow")
        loc2 = [231.788, -27.154, 313.011, 342.317, 0.0, 683.702] #Sealer
        peeler = [264.584, -29.413, 284.376, 372.338, 0.0, 651.621]
        pos2= [197.185, 59.736, 90.509, 566.953, 82.069, -65.550] #OT2

        # self.pf400.move_joints(loc2)
        self.pf400.transfer(peeler, loc2 ,"narrow","narrow")

    def joint_state_publisher_callback(self):
        while rclpy.ok():
            joint_states = self.pf400.refresh_joint_state()
            pf400_joint_msg = JointState()
            pf400_joint_msg.header = Header()
            pf400_joint_msg.header.stamp = self.get_clock().now().to_msg()
            pf400_joint_msg.name = ['J1', 'J2', 'J3', 'J4', 'J5','J5_mirror', 'J6']
            pf400_joint_msg.position = joint_states
            print(joint_states)

            # pf400_joint_msg.position = [0.01, -1.34, 1.86, -3.03, 0.05, 0.05, 0.91]
            pf400_joint_msg.velocity = []
            pf400_joint_msg.effort = []

            self.joint_publisher.publish(pf400_joint_msg)
            # self.get_logger().info('Publishing: "%s"' % pf400_joint_msg.position)


def main(args=None):
    rclpy.init(args=args)

    pf400_joint_state_publisher = PF400DescriptionClient()

    rclpy.spin(pf400_joint_state_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pf400_joint_state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()