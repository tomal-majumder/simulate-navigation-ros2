import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import math

class FSMNavigator(Node):
    def __init__(self):
        super().__init__('fsm_navigator')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.camera_subscriber = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.closest_obstacle = float('inf')
        self.red_detected = False
        self.red_x = 0

        self.state = 'IDLE'
        self.get_logger().info("🧠 Smarter FSM Navigation Node Started")

    def scan_callback(self, msg):
        # Read front-facing obstacle distances
        front_ranges = msg.ranges[len(msg.ranges)//2 - 10 : len(msg.ranges)//2 + 10]
        self.closest_obstacle = min(front_ranges)

    def camera_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Camera conversion failed: {e}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] > 0:
                self.red_detected = True
                self.red_x = int(M['m10']/M['m00'])
                self.get_logger().info("🔴 Red object detected")
                return

        self.red_detected = False
        self.get_logger().info("❌ Red object not found")

    def timer_callback(self):
        msg = Twist()
        linear_speed = 0.2
        angular_speed = 0.5

        if self.closest_obstacle < 0.3:
            self.state = 'AVOID_OBSTACLE'
        elif self.red_detected:
            self.state = 'TRACK_RED'
        else:
            self.state = 'WANDER'

        if self.state == 'AVOID_OBSTACLE':
            msg.linear.x = 0.0
            msg.angular.z = angular_speed  # Turn to avoid
            self.get_logger().info("🚧 Obstacle detected - Turning")

        elif self.state == 'TRACK_RED':
            center = 320  # Assuming image width is 640px
            error = self.red_x - center
            msg.linear.x = 0.1
            msg.angular.z = -error / 300  # Proportional control
            self.get_logger().info("🎯 Tracking red object")

        elif self.state == 'WANDER':
            msg.linear.x = linear_speed
            msg.angular.z = 0.0
            self.get_logger().info("🚶 Wandering")

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FSMNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

