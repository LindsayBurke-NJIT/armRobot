#!/usr/bin/env python3

# ROS 2 Python Node for controlling the Hiwonder ArmPi Pro base velocity.
# This script publishes a constant linear velocity command to the /cmd_vel topic,
# causing the robot to drive forward until stopped.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ArmPiDriverNode(Node):
    """
    A ROS 2 Node that publishes a constant forward velocity command.
    """
    def __init__(self):
        # Initialize the Node with a unique name
        super().__init__('armpi_forward_driver_ros2')
        self.get_logger().info("ArmPi Pro ROS 2 Driver Node Initialized.")

        # --- Configuration ---
        self.LINEAR_SPEED = 0.2  # Forward velocity in m/s
        self.PUBLISH_RATE_HZ = 10  # Publishing frequency in Hz
        self.topic_name = '/cmd_vel'

        # 1. Create a Publisher
        # Publishes Twist messages to the standard velocity topic.
        self.velocity_publisher = self.create_publisher(
            msg_type=Twist,
            topic=self.topic_name,
            qos_profile=10  # QoS profile depth
        )

        # 2. Create a Twist message for forward movement
        self.forward_vel_msg = Twist()
        # Linear X: Forward/Backward velocity (m/s). Positive is forward.
        self.forward_vel_msg.linear.x = self.LINEAR_SPEED
        self.forward_vel_msg.linear.y = 0.0
        self.forward_vel_msg.linear.z = 0.0
        # Angular components (no rotation needed for straight movement)
        self.forward_vel_msg.angular.x = 0.0
        self.forward_vel_msg.angular.y = 0.0
        self.forward_vel_msg.angular.z = 0.0

        # 3. Create a Timer to run the publishing function periodically
        self.timer = self.create_timer(
            timer_period_sec=1.0 / self.PUBLISH_RATE_HZ,
            callback=self.publish_command
        )
        
        self.get_logger().info(
            f"Starting ArmPi Pro forward drive (Topic: {self.topic_name}, Linear X: {self.LINEAR_SPEED:.2f} m/s)."
        )

    def publish_command(self):
        """
        The callback function run by the timer to send the velocity command.
        """
        self.velocity_publisher.publish(self.forward_vel_msg)

    def stop_robot(self):
        """
        Publishes a zero velocity command to stop the robot cleanly upon shutdown.
        """
        stop_vel_msg = Twist()
        stop_vel_msg.linear.x = 0.0
        # Publish the stop command multiple times for safety
        for _ in range(5):
             self.velocity_publisher.publish(stop_vel_msg)
             # Wait briefly to ensure the command is received
             rclpy.spin_once(self, timeout_sec=0.1) 
        
        self.get_logger().info("ArmPi Pro forward drive stopped.")


def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)
    
    # Create the node instance
    armpi_driver_node = ArmPiDriverNode()

    try:
        # Keep the node running until it is manually shut down (e.g., Ctrl+C)
        rclpy.spin(armpi_driver_node)
    except KeyboardInterrupt:
        # Handle the interrupt and stop the robot before destruction
        armpi_driver_node.stop_robot()
    finally:
        # Clean up and shut down the node and rclpy
        armpi_driver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()