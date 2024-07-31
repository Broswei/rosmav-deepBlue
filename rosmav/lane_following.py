import rclpy
from rclpy.node import Node
import rosmav.lane_detection as ld
from std_msgs.msg import Int16
import numpy as np


from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2



class BLUEROV2LANEFOLLOWING(Node):
    def __init__(self):
        super().__init__("lane_following")
        
        self.cvb = CvBridge()
        self.curr_heading = 0

        self.desired_heading_publisher = self.create_publisher(
            Int16, "bluerov2/desired_heading", 10
        )

        self.heading_subscriber = self.create_subscription(
            Int16, "bluerov2/heading", self.heading_callback, 10
        )

        self.subscription = self.create_subscription(
            Image, "bluerov2/camera", self.image_callback, 10
        )


    def heading_callback(self, msg):
        self.curr_heading = msg.data

    def get_heading(self, slopes):

        # slopes = ld.get_slopes_intercepts(image)  # This returns a list of slopes
        if len(slopes) != 0:
            center_slope = max(abs(slope) for slope in slopes)

            return (-np.degrees(np.arctan(center_slope)) + self.curr_heading)
        return 0


    def desired_heading_callback(self, image):
        # Calculating the desired with the other functions

        image, lines = ld.detect_lines(image, 300, 350, 5, 100, 150)

        # self.get_logger().info(f"{lines}")
        image = ld.draw_lines(image, lines)
        cv2.imwrite("image.png", image) 
        slopes, intercepts = ld.get_slopes_intercepts(lines)

        new_desired_heading = Int16()
        
        new_desired_heading.data = int(self.get_heading(slopes))


        # publish the desired heacding for the PID
        
        self.desired_heading_publisher.publish(new_desired_heading)
        self.get_logger().info(f"Published desired heading: {new_desired_heading} deg.")


    def image_callback(self, msg: Image):
        image = self.cvb.imgmsg_to_cv2(msg, "bgr8")

        # Save the image
        # # Get the dimensions of the image
        # height, width, channels = image.shape

        # # # Log the dimensions
        # self.get_logger().info(f"Image dimensions: width={width}, height={height}, channels={channels}")
        

        # 
        self.desired_heading_callback(image)





def main(args=None):
    rclpy.init(args=args)
    node = BLUEROV2LANEFOLLOWING()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()