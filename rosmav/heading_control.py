#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Int16
import numpy as np
from sensor_msgs.msg import Imu

from rosmav.pid import PID

class HeadingPIDNode(Node):
    previous_error = 0.0
    integral = 0.0
    desired_heading: int = None
    previous_heading: int = None
    previous_time = 0.0
    curr_time = 0.0
    derivative = 0.0

    def __init__(self):
        super().__init__("heading_control")

        self.declare_parameter("Kp", 1.5)
        self.Kp = self.get_parameter("Kp").value
        self.declare_parameter("Ki", 0.0)
        self.Ki = self.get_parameter("Ki").value
        self.declare_parameter("Kd", 0.0)
        self.Kd = self.get_parameter("Kd").value
        self.declare_parameter("max_integral", 1.0)
        self.max_integral = self.get_parameter("max_integral").value

        self.pid = PID(self.Kp, self.Ki, self.Kd)

        self.heading_sub = self.create_subscription(
            Int16, "bluerov2/heading", self.heading_callback, 10
        )
        self.desired_heading_sub = self.create_subscription(
            Int16, "bluerov2/desired_heading", self.desired_heading_callback, 10
        )

        self.gyro_sub = self.create_subscription(
            Imu, "bluerov/imu", self.gyro_callback, 10
        )

        self.manual_control_pub = self.create_publisher(
            ManualControl, "bluerov2/manual_control", 10
        )
        

    def heading_callback(self, msg):
        heading = msg.data
        self.get_logger().debug(f"Heading: {heading}")

        if self.desired_heading is None:
            return

        error = self.desired_heading.data - heading

        # angle wrap the error here

        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        
        self.get_logger().info(f"error: {error}")
        # end of angle wrap

        # if self.previous_heading is None:
        #     self.previous_heading = heading
        #     return

        # dt = self.curr_time - self.previous_time

        # # Propotional term
        # propotional = self.Kp * error

        # # Integral term
        # self.integral += self.Ki * error * dt
        # self.integral = min(max(self.integral, -self.max_integral), self.max_integral)

        # # Derivative term
        # derivative = self.Kd * (error - self.previous_error) / dt

        # # Update previous values
        # self.previous_error = error
        # self.previous_heading = heading

        

        yaw = self.pid.update(error)

        yaw = np.clip(yaw, -100, 100)

        manual_control_msg = ManualControl()
        manual_control_msg.r = yaw
        self.manual_control_pub.publish(manual_control_msg)


        self.get_logger().info(f"\nTarget Heading: {self.desired_heading.data:.3f}\n Current Heading:{heading}   \nPower: {yaw:.3f}")

    def desired_heading_callback(self, msg):
        self.desired_heading = msg
        self.get_logger().debug(f"Desired heading: {self.desired_heading}")

    def gyro_callback(self, msg):
        heading: Imu = msg
        self.previous_time = self.curr_time
        self.curr_time = heading.header.stamp.sec + heading.header.stamp.nanosec * 1e-9


    def function_f(self, x):
        return np.sin(x * ((np.pi)/180) )
    
    def function_h(self, x):
        return (100 * np.sin(x * ((np.pi) / 360)))

    def combination(self, x):
        return np.abs(self.function_h(x))*(np.sign(self.function_f(x)))


def main(args=None):
    rclpy.init(args=args)
    node = HeadingPIDNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()