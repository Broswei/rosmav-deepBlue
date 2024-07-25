#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from mavros_msgs.msg import ManualControl, Altitude 

from rosmav.pid import PID
import numpy as np


class DepthControl(Node):
    desired_depth = None

    def __init__(self):
        '''
        Initializes the publishers and subscribers etc etc etc
        '''
        super().__init__("depth_control")

        self.desired_depth = None

        # Coefficients k_p  k_i  k_d
        self.pid = PID(3.0, 0, 8.0)

        # Publisher of the manual_control to move the rov
        self.desired_depth_pub = self.create_publisher(
            ManualControl, "bluerov2/manual_control", 10
        )

        # Subscriber to the current depth
        self.depth_control_sub = self.create_subscription(
            Altitude,"bluerov2/depth", self.depth_callback, 10
        )

        # Subscriber to be able to change the desired_depth with a manual publish cmd
        self.desired_depth_sub = self.create_subscription(
            Altitude,"bluerov2/desired_depth", self.desired_depth_callback, 10
        )

        self.get_logger().info("starting node...")

    def desired_depth_callback(self, msg):
        """
        This listens for a manual published message controlling the desired depth
        """
        self.desired_depth = msg.relative

    def depth_callback(self, msg):
        """
        This is the callback method for the depth. This function takes the current depths and caluculates power based on our PID Controller.
        Clips power into the range of -100 to 100 and is converted into manual control message type (-1000 to 1000). Returns target, current depths,
        and the power calculated by the PID.
        """
        # Checking if a desired_depth has been inputed
        if  self.desired_depth is None:
            return
        
        manual_msg = ManualControl()

        # Error is the delta of the current depth and the desired depth
        error = self.desired_depth - msg.relative

        # Calls the update method in pid
        power = self.pid.update(error)

        # Prints that the power value will be clipped if it is outside valid range
        if power < -100 or power > 100:
            print("Power value out of range. Clipping...")

        # Clips power to make it within bounds
        power = np.clip(power, -100, 100)

        # Sets the timestamp of the message being published
        manual_msg.header.stamp = self.get_clock().now().to_msg()

        # Setting the z-axis which is up and publishing
        manual_msg.z = -10*power
        self.desired_depth_pub.publish(manual_msg)


        self.get_logger().info(f"\nTarget Depth: {self.desired_depth:.3f}\n       Depth: {msg.relative:.3f}\n       Power: {power:.3f}")
        
# Standard main method
def main(args=None):
    rclpy.init(args=args)
    node = DepthControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()