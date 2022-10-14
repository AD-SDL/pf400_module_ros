import rclpy
from rclpy.node import Node
# from rclpy.clock import clock

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class P400JointPublisher(Node):

    def __init__(self):
        super().__init__('pf400_joint_state_publisher')
        
        self.publisher_ = self.create_publisher(JointState,'joint_states',10)
        self.publish_joint_callback = self.create_timer(0.5, self.joint_state_publisher_callback)

    def joint_state_publisher_callback(self):
        while rclpy.ok():
            joint_stastes = [0.01,-1.34,1.86,-3.03,0.05,0.05,0.91]
            pf400_joint_msg = JointState()
            pf400_joint_msg.header = Header()
            pf400_joint_msg.header.stamp = self.get_clock().now().to_msg()
            pf400_joint_msg.name = ['J1', 'J2', 'J3', 'J4', 'J5','J5_mirror', 'J6']
            pf400_joint_msg.position = [0.01, -1.34, 1.86, -3.03, 0.05, 0.05, 0.91]
            pf400_joint_msg.velocity = []
            pf400_joint_msg.effort = []

            self.publisher_.publish(pf400_joint_msg)
            self.get_logger().info('Publishing: "%s"' % pf400_joint_msg.position)


def main(args=None):
    rclpy.init(args=args)

    pf400_joint_state_publisher = P400JointPublisher()

    rclpy.spin(pf400_joint_state_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pf400_joint_state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()