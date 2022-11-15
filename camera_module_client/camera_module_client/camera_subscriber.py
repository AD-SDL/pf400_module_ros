
import cv2  # OpenCV library
import rclpy  # Python Client Library for ROS 2
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from rclpy.node import Node  # Handles the creation of nodes
from std_msgs.msg import String
from sensor_msgs.msg import Image  # Image is the message type
from rclpy.qos import qos_profile_sensor_data
from wei_services.srv import WeiActions  


class CameraSubscriberNode(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """

    def __init__(self, NODE_NAME = "Camera_Subscriber_Node"):

        """
        Class constructor to set up the Camera node
        """

        super().__init__(NODE_NAME)

        # We will publish a message every 0.1 seconds
        timer_period = 0.1  # seconds
        # State publisher
        self.state = "UNKNOWN"
        self.statePub = self.create_publisher(String, NODE_NAME + '/state', 10)
        self.stateTimer = self.create_timer(timer_period, self.stateCallback)

        self.cameraSub = self.create_subscription(Image, "Camera_Publisher_Node/video_frames", self.cameraSubCallback, qos_profile_sensor_data)
        self.cameraSub
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
    
    def stateCallback(self):
        '''
        Publishes the state to the 'state' topic. 
        '''

        msg = String()
        msg.data = 'State: %s' % self.state
        self.statePub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.state = "READY"
        # self.cameraSubCallback()

    def cameraSubCallback(self, data):
            """
            Callback function.
            """
            # Display the message on the console
            self.get_logger().info('Receiving video frame')
        
            # Convert ROS Image message to OpenCV image
            current_frame = self.br.imgmsg_to_cv2(data)
            
            # Display image
            cv2.imshow("camera", current_frame)
            cv2.waitKey(1)
   

def main(args=None):  # noqa: D103

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    node = CameraSubscriberNode()

    # Spin the node so the callback function is called.
    rclpy.spin(node)
    node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
