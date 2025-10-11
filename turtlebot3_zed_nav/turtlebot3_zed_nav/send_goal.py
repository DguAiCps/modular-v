#!/usr/bin/env python3
"""
Send navigation goal to Nav2

Usage:
    ros2 run turtlebot3_zed_nav send_goal <x> <y> <yaw_degrees>
    
Example:
    ros2 run turtlebot3_zed_nav send_goal 2.0 1.5 90
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

class NavGoalSender(Node):
    def __init__(self):
        super().__init__('nav_goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
    def send_goal(self, x, y, yaw_degrees):
        # Wait for action server
        self.get_logger().info('Waiting for Nav2 action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available!')
            return False
        
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Orientation (convert yaw to quaternion)
        yaw_rad = math.radians(float(yaw_degrees))
        goal_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        
        self.get_logger().info(f'Sending goal: x={x}, y={y}, yaw={yaw_degrees}°')
        
        # Send goal
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False
        
        self.get_logger().info('Goal accepted! Robot is navigating...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        self.get_logger().info('Navigation completed!')
        return True
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f}m',
            throttle_duration_sec=2.0
        )

def main():
    if len(sys.argv) < 4:
        print("Usage: ros2 run turtlebot3_zed_nav send_goal <x> <y> <yaw_degrees>")
        print("Example: ros2 run turtlebot3_zed_nav send_goal 2.0 1.5 90")
        sys.exit(1)
    
    x = sys.argv[1]
    y = sys.argv[2]
    yaw = sys.argv[3]
    
    rclpy.init()
    node = NavGoalSender()
    
    try:
        node.send_goal(x, y, yaw)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
