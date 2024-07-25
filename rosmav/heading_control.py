#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Float32
import numpy as np


class HeadingPIDNode(Node):
    previous_error = 0.0
    integral = 0.0
    desired_depth: Float32 = None
    previous_depth: Float32 = None

    def __init__(self):
        super().__init__("heading_pid_control")

        self.declare_parameter("Kp", -50.0)
        self.Kp = self.get_parameter("Kp").value
        self.declare_parameter("Ki", 0.0)
        self.Ki = self.get_parameter("Ki").value
        self.declare_parameter("Kd", 30.0)
        self.Kd = self.get_parameter("Kd").value
        self.declare_parameter("max_integral", 1.0)
        self.max_integral = self.get_parameter("max_integral").value
        self.declare_parameter("max_throttle", 100.0)
        self.max_throttle = self.get_parameter("max_throttle").value

        self.heading_sub = self.create_subscription(
            Float32, "bluerov2/heading", self.heading_callback, 10
        )
        self.desired_heading_sub = self.create_subscription(
            Float32, "bluerov2/desired_heading", self.desired_heading_callback, 10
        )

        self.manual_control_pub = self.create_publisher(
            ManualControl, "bluerov2/manual_control", 10
        )

    def heading_callback(self, msg):
        heading: Float32 = msg
        self.get_logger().debug(f"Depth: {heading}")

        if self.desired_heading is None:
            return

        error = self.desired_heading - heading
        


        if self.previous_depth is None:
            self.previous_depth = heading
            return

        dt = (
            heading.header.stamp.sec
            + heading.header.stamp.nanosec * 1e-9
            - self.previous_depth.header.stamp.sec
            - self.previous_depth.header.stamp.nanosec * 1e-9
        )

        # Propotional term
        propotional = self.Kp * error

        # Integral term
        self.integral += self.Ki * error * dt
        self.integral = min(max(self.integral, -self.max_integral), self.max_integral)

        # Derivative term
        derivative = self.Kd * (error - self.previous_error) / dt

        # Update previous values
        self.previous_error = error
        self.previous_depth = heading

        throttle = propotional + self.integral + derivative
        throttle = min(max(throttle, -self.max_throttle), self.max_throttle)

        manual_control_msg = ManualControl()
        manual_control_msg.r = throttle
        self.manual_control_pub.publish(manual_control_msg)

    def desired_heading_callback(self, msg):
        self.desired_heading = msg
        self.get_logger().debug(f"Desired depth: {self.desired_heading}")

    def function_f(x):
        return np.sin(x * ((np.pi)/180) )
    
    def function_h(x):
        return (100 * np.sin(x * ((np.pi) / 360)))

    def combination(x):
        


def main(args=None):
    rclpy.init(args=args)
    HeadingPIDNode = HeadingPIDNode()

    try:
        rclpy.spin(HeadingPIDNode)
    except KeyboardInterrupt:
        pass
    finally:
        HeadingPIDNode.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()