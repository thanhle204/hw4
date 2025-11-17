#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.qos import qos_profile_sensor_data
import math


class LocomotionBrain(Node):

    def __init__(self):
        super().__init__('turtlebot3_brain')

        self.robot_state = "searching"
        self.get_logger().info(f"Starting in state: {self.robot_state}")

        self.current_obstacle_distance = 99.0
        self.is_tag_visible = False
        self.tag_count = 0

        # Tunable parameters
        self.obstacle_threshold = 0.5
        self.forward_speed = 0.12
        self.turn_speed = 0.5
        self.scan_speed = 0.7
        self.scan_duration_sec = 3.0   # Reduced from 10 seconds
        self.scan_start_time = None

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data)

        self.tag_subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.tag_callback,
            10)

        self.decision_timer = self.create_timer(0.1, self.decision_loop)

    def scan_callback(self, msg):
        """Updates distance to obstacle."""
        val = msg.ranges[0]
        if val == float('inf') or math.isnan(val):
            self.current_obstacle_distance = 99.0
        else:
            self.current_obstacle_distance = val

    def tag_callback(self, msg):
        """Updates tag visibility."""
        if len(msg.detections) > 0:
            if not self.is_tag_visible:
                self.tag_count += 1
                self.get_logger().info(f"[TAG #{self.tag_count}] ID {msg.detections[0].id}")
            self.is_tag_visible = True
        else:
            self.is_tag_visible = False

    def decision_loop(self):
        is_blocked = self.current_obstacle_distance < self.obstacle_threshold

        # ===== STATE TRANSITIONS =====
        if self.robot_state == "searching":
            if self.is_tag_visible:
                self.robot_state = "scanning"
                self.scan_start_time = self.get_clock().now()
                self.get_logger().info("--> SCANNING")
            elif is_blocked:
                self.robot_state = "avoiding"
                self.get_logger().info("--> AVOIDING")

        elif self.robot_state == "avoiding":
            if not is_blocked:
                self.robot_state = "searching"
                self.get_logger().info("--> SEARCHING")

        elif self.robot_state == "scanning":
            elapsed = (self.get_clock().now() - self.scan_start_time).nanoseconds / 1e9
            if elapsed > self.scan_duration_sec:
                self.robot_state = "searching"
                self.get_logger().info("--> SEARCHING (scan done)")

        # ===== ACTIONS =====
        twist_msg = Twist()

        if self.robot_state == "searching":
            twist_msg.linear.x = self.forward_speed
            twist_msg.angular.z = 0.0

        elif self.robot_state == "avoiding":
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = self.turn_speed  # consistent turn (no random)

        elif self.robot_state == "scanning":
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = self.scan_speed

        self.publisher_.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocomotionBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.publisher_.publish(stop)
        node.get_logger().info("Shutting down, stopping robot.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
