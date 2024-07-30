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

        self.desired_heading_publisher = self.create_publisher(
            Int16, "bluerov2/desired_heading", 10
        )


        self.subscription = self.create_subscription(
            Image, "bluerov2/camera", self.image_callback, 10
        )


    def get_heading(self, image):

        slopes = ld.return_slopes(image)  # This returns a list of slopes

        # if len(slopes) == 0: 
        #     return
        
        center_slope = max(abs(slope) for slope in slopes)

        return np.degrees(np.arctan(center_slope))  


    def desired_heading_callback(self, image):
        # Calculating the desired with the other functions

        try:
            new_desired_heading = self.get_heading(ld.detect_lanes(image))


            # publish the desired heading for the PID
            
            self.desired_heading_publisher.publish(new_desired_heading)
            self.get_logger().info(f"Published desired heading: {new_desired_heading} deg.")
        except: 
            self.get_logger().info(f"No lanes detected")


    def image_callback(self, msg: Image):
        image = self.cvb.imgmsg_to_cv2(msg, "bgr8")

        # Save the image
        cv2.imwrite("image.png", image) 
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