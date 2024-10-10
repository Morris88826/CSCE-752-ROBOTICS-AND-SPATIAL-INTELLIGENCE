#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node') # Node name
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) # Publisher: (message type, topic name, queue size)
        self.get_logger().info('Circular node has been started')
        self.create_timer(1, self.run)

    def run(self):
        twist = Twist()
        twist.linear.x = 1.0 # set the linear speed of the turtle in the x-axis direction (usually forward)
        twist.angular.z = 1.0 # set the angular speed of the turtle in the z-axis direction (usually rotation), positive for counter-clockwise rotation (left)
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        self.get_logger().info('Circular node has been stopped')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    control_node = ControlNode()
    rclpy.spin(control_node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()