import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from enum import Enum

class State(Enum):
    EXPLORE = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    STOP = 4

class FSMNavigator(Node):
    def __init__(self):
        super().__init__('fsm_navigator')
        self.state = State.EXPLORE
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("FSM Navigator Started.")

    def scan_callback(self, msg):
        front = min(msg.ranges[0:15] + msg.ranges[-15:])
        left = min(msg.ranges[60:90])
        right = min(msg.ranges[-90:-60])

        cmd = Twist()

        # FSM transitions
        if self.state == State.EXPLORE:
            if front < 0.4:
                self.state = State.TURN_LEFT if left > right else State.TURN_RIGHT
        elif self.state in (State.TURN_LEFT, State.TURN_RIGHT):
            if front > 0.5:
                self.state = State.EXPLORE

        # Actions
        if self.state == State.EXPLORE:
            cmd.linear.x = 0.2
        elif self.state == State.TURN_LEFT:
            cmd.angular.z = 0.5
        elif self.state == State.TURN_RIGHT:
            cmd.angular.z = -0.5
        elif self.state == State.STOP:
            cmd.linear.x = 0.0

        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = FSMNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
