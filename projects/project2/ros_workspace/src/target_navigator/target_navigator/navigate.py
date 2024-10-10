#!/usr/bin/env python3
import rclpy
import enum
import numpy as np
from collections import deque
from rclpy.node import Node
from std_srvs.srv import Empty
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist, Pose2D


class RobotAction(enum.Enum):
    IDLE = 0
    ROTATE = 1
    MOVE = 2
    
class Navigate(Node):
    def __init__(self):
        super().__init__('navigate') # Node name
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) # Publisher: (message type, topic name, queue size)
        self.robot_pose_sub = self.create_subscription(Pose2D, '/pose', self.robot_pose_callback, 10) # Subscriber: (message type, topic name, callback function, queue size)
        self.target_sub = self.create_subscription(PointCloud, '/unvisited_targets', self.unvisited_targets_callback, 10)
        self.get_logger().info('Navigate node has been started')
        self.create_timer(0.1, self.timer_callback)

        self.origin = None
        self.route = None
        self.completed = False
        self.robot_current_state = None
        self.state = RobotAction.IDLE

        # Declare parameters
        self.declare_parameter('method', 'default')
        self.declare_parameter('t_tol', 0.001)
        self.declare_parameter('d_tol', 0.1)

        self.get_logger().info('Navigate node has been initialized')

        # Get parameters (either from external source or use the default)
        self.method = self.get_parameter('method').value
        assert self.method in ['default', 'approx']
        self.theta_tolerance = self.get_parameter('t_tol').get_parameter_value().double_value
        self.distance_tolerance = self.get_parameter('d_tol').get_parameter_value().double_value

        self.reset_simulation()


    def _held_karp(self, points):
        # Implement a path planning algorithm to find the shortest route to visit all targets
        n = len(points)
        dist = [[0] * n for _ in range(n)]
        for i in range(n):
            for j in range(n):
                dist[i][j] =  np.sqrt((points[i][0] - points[j][0]) ** 2 + (points[i][1] - points[j][1]) ** 2)
        dist = np.array(dist)

        dp = [[float('inf')] * (1 << n) for _ in range(n)]
        dp = np.array(dp)
        dp[0][1] = 0

        for mask in range(1, 1 << n):
            for i in range(1, n): # 0 is the starting point
                if mask & (1 << i): # i in the subset
                    for j in range(n):
                        if mask & (1 << j): # j in the subset
                            dp[i][mask] = min(dp[i][mask], dp[j][mask ^ (1 << i)] + dist[j][i])

        mask = (1 << n) - 1 # [1, 1, 1, ..., 1]
        last = np.argmin(dp[:, mask]) # the last point
        path = deque([last])

        # backtracking
        while mask != 1:
            for i in range(n):
                if mask & (1 << i) and dp[last][mask] == dp[i][mask ^ (1 << last)] + dist[i][last]:
                    path.appendleft(i)
                    mask ^= (1 << last)
                    last = i
                    break

        return path
    
    def _nearest_neighbor(self, points):
        # Implement the nearest neighbor algorithm to find the shortest route to visit all targets
        raise NotImplementedError
        return path

    def find_route(self, points, method='default'):
        print("Method: ", method)
        if method == 'default':
            self.get_logger().info('Using default method: Held-Karp')
            return self._held_karp(points)
        elif method == 'approx':
            self.get_logger().info('Using approximate method: Nearest Neighbor')
            return self._nearest_neighbor(points)
    

    def reset_simulation(self):
        reset_srv = self.create_client(Empty, '/reset')
        if reset_srv.wait_for_service(timeout_sec=3.0):
            reset_srv.call_async(Empty.Request())

    def robot_pose_callback(self, msg):
        if self.origin is None and (msg.x, msg.y) == (0, 0):
            self.origin = (msg.x, msg.y)
        self.robot_current_state = (msg.x, msg.y, msg.theta)

    def unvisited_targets_callback(self, msg):
        unvisited_targets = msg.points
        
        if self.route is None and self.origin is not None:
            points = [self.origin]
            for target in unvisited_targets:
                points.append((target.x, target.y))

            route_idx = self.find_route(points, method=self.method)
            self.route = [points[i] for i in route_idx]
            self.route = deque(self.route)
            print("Route: ", self.route)

        if len(unvisited_targets) == 0 and not self.completed:
            self.get_logger().info('All targets have been visited')
            self.completed = True


        if self.route is not None and not self.completed:
            if len(unvisited_targets) < len(self.route):
                self.route.popleft()
                self.get_logger().info('Next target: {}'.format(self.route[0]))


    def timer_callback(self):
        if not self.route or self.robot_current_state is None:
            return
        
        if self.state == RobotAction.IDLE:
            self.target = self.route[0]
            self.state = RobotAction.ROTATE           
        
        elif self.state == RobotAction.ROTATE:
            x, y, theta = self.robot_current_state
            angle = np.arctan2(self.target[1] - y, self.target[0] - x)
            
            route1 = angle - theta
            route2 = route1 + np.pi * 2 if route1 < 0 else route1 - np.pi * 2
            
            angle = route1 if abs(route1) < abs(route2) else route2
            
            if abs(angle) < self.theta_tolerance:
                self.cmd_vel_pub.publish(Twist())
                self.state = RobotAction.MOVE
            else:
                msg = Twist()
                msg.angular.z = angle
                self.cmd_vel_pub.publish(msg)
        
        elif self.state == RobotAction.MOVE:
            x, y, theta = self.robot_current_state
            self.distance = np.sqrt((self.target[0] - x) ** 2 + (self.target[1] - y) ** 2)
            
            if self.distance < self.distance_tolerance:
                self.cmd_vel_pub.publish(Twist())
                self.state = RobotAction.IDLE
                self.target = None
                self.distance = 0
                self.dist_left = None
            else:
                msg = Twist()
                msg.linear.x = self.distance
                self.cmd_vel_pub.publish(msg)


    def stop(self):
        self.get_logger().info('Navigate node has been stopped')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    navigate_node = Navigate()
    rclpy.spin(navigate_node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()