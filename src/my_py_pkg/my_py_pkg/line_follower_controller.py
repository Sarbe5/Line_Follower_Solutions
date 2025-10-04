import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np

class LineFollowerControllerNode(Node):
    def __init__(self):
        super().__init__('line_follower_controller')

        self.subscription = self.create_subscription(
            Image, "camera/image_raw", self.callback, 10
        )
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.br = CvBridge()

        # Proportional control parameters
        self.Kp = 2.0  # Proportional gain

    def callback(self, msg):

        img = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img = cv2.resize(img, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_CUBIC)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        twist = Twist()

        linear_x = 0.4

        h_min, h_max = 0, 179      # allow all hues
        s_min, s_max = 0, 255      # allow all saturation
        v_min, v_max = 0, 20       # very low brightness = black

        # Making the mask
        low = np.array([h_min, s_min, v_min])
        upp = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(img_hsv, low, upp)

        # Region of interest (keep the lower 1/5 of the image)
        height, width = mask.shape
        roi = mask.copy()

        # Calculate centroid of the region of interest
        M = cv2.moments(roi)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            error = (cX - width/2)/(width/2)   # normalized -> range roughly [-1,1]

            # Proportional control to adjust the angular velocity
            angular_z = -self.Kp * error

            # Move the robot based on the distance from the center of the region of interest
            twist.linear.x = linear_x
            twist.angular.z = angular_z

        else:
            twist.linear.x = 0.0
            twist.angular.z = 2.0

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
