#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Color

class CircularNode(Node):
    def __init__(self):
        super().__init__('circular_node') # Node name
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) # Publisher: (message type, topic name, queue size)
        self.color_sensor_sub = self.create_subscription(Color, '/turtle1/color_sensor', self.color_sensor_callback, 10) # Subscriber: (message type, topic name, callback function, queue size)
        self.get_logger().info('Circular node has been started')
        self.create_timer(1, self.run)

    def run(self):
        twist = Twist()
        twist.linear.x = 1.0 # set the linear speed of the turtle in the x-axis direction (usually forward)
        twist.angular.z = 1.0 # set the angular speed of the turtle in the z-axis direction (usually rotation), positive for counter-clockwise rotation (left)
        self.cmd_vel_pub.publish(twist)

    def color_sensor_callback(self, msg):
        self.get_logger().info('Color sensor callback: R={}, G={}, B={}'.format(msg.r, msg.g, msg.b))

    def stop(self):
        self.get_logger().info('Circular node has been stopped')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    circular_node = CircularNode()
    rclpy.spin(circular_node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()