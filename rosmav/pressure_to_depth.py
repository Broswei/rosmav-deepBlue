#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure as Pressure
from mavros_msgs.msg import Altitude

class PressureConverter(Node):

    def __init__(self):
        """
        Initializes the node.
        """

        super().__init__("depth_node")

        self._pressure_subscriber = self.create_subscription(
            Pressure,
            "bluerov2/pressure",
            self.pressure_callback,
            10
        )        

        self.depth = self.create_publisher(
            Altitude,
            "bluerov2/depth",
            10
        )

        self.get_logger().info("starting node...")

    def pressure_callback(self, msg:Pressure):
        """
        Callback for the pressure subscription. Calculates the pressure purely from water --> converts to depth. Takes into account
        the offset for a ROV that we tested to ensure that surface depth is close to 0.00. Logs pressure and depth to 3 decimal places.
        """
        depth = Altitude() 
        pressure = msg.fluid_pressure - 101325 # subtracted the surface pressure of 1 atm --> 101325 Pa
        depth.header.stamp = self.get_clock().now().to_msg()
        depth.relative = self.calculate_depth(pressure) - 0.16 # constant subtraction required between different ROVs, -0.22 for tight spool, -0.12 for loose spool
        self.depth.publish(depth)
        self.get_logger().info(f"Pressure: {pressure:.3f}")
        self.get_logger().info(f"Depth: {depth.relative:.3f}")

    def calculate_depth(self, pressure):
        """
        Calculates depth from pressure, rho, and gravitational acceleration constant.
        """
        water_density = 1000
        g = 9.81
        return ((pressure) / (water_density * g))


def main(args=None):
    rclpy.init(args=args)
    node = PressureConverter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()