#! /usr/bin/env python3
"""Camera node"""

import cv2  # OpenCV library
import rclpy  # Python Client Library for ROS 2
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from rclpy.node import Node  # Handles the creation of nodes
from std_msgs.msg import String
from sensor_msgs.msg import Image  # Image is the message type
from wei_services.srv import WeiActions  


class cameraNode(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """

    def __init__(self, NODE_NAME = "Camera_Node"):

        """
        Class constructor to set up the Camera node
        """

        super().__init__(NODE_NAME)

        # We will publish a message every 0.1 seconds
        timer_period = 0.2  # seconds
        # State publisher
        self.state = "UNKNOWN"
        self.statePub = self.create_publisher(String, NODE_NAME + '/state', 10)
        self.stateTimer = self.create_timer(timer_period, self.stateCallback)

        # Initiate the Node class's constructor and give it a name

        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.cameraPub = self.create_publisher(Image, "video_frames", 10)

        self.action_handler = self.create_service(WeiActions, NODE_NAME + "/action_handler", self.actionCallback)

        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(0)

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
        self.get_logger().info("Capturing image")
        self.cameraCallback()

    def cameraCallback(self):
        """Callback function.
        This function gets called every 0.1 seconds.
        """

        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.

        ret, frame = self.cap.read()
        if ret:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            self.cameraPub.publish(self.br.cv2_to_imgmsg(frame))

        # Display the message on the console
        self.get_logger().info("Publishing video frame")

    def actionCallback(self, request, response):
        '''
        The actionCallback function is a service that can be called to execute the available actions the robot
        can preform.
        '''
        print("Action call....")
        
        if request.action_handle == "capture_image":

            self.state = "BUSY"
            self.stateCallback()
            vars = eval(request.vars)
            print(vars)

        self.get_logger().info("Capturing image")
        self.cameraCallback()
        self.get_logger().info("Plate image saved.")

        self.state = "COMPLETED"

        return response

def main(args=None):  # noqa: D103

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    node = cameraNode()

    # Spin the node so the callback function is called.
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
