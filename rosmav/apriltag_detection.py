import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from dt_apriltags import Detector
import numpy as np
import time

class AprilTagDetection(Node):

    fx = 1060.7
    fy = 1060.7
    error = 0

    def __init__(self):
        super().__init__("april_tags")
        
        self.cvb = CvBridge()

        self.at_detector = Detector(families='tag36h11',
                       nthreads=2,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

        self.desired_heading_publisher = self.create_publisher(
            Int16, "bluerov2/desired_heading", 10
        )

        self.heading_subscriber = self.create_subscription(
            Int16, "bluerov2/heading", self.heading_callback, 10
        )

        self.subscription = self.create_subscription(
            Image, "bluerov2/camera", self.image_callback, 10
        )

        self.command_pub = self.create_publisher(
            OverrideRCIn, "bluerov2/override_rc", 10
        )


    def heading_callback(self, msg):
        desired = Int16()
        desired.data = msg.data + self.error
        self.desired_heading_publisher.publish(desired)


    def image_callback(self, msg: Image):

        image = self.cvb.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite("image.png", image) 

        # Save the image

        # Get the dimensions of the image
        img_width = image.shape[1]
        img_height = image.shape[0]
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        tags = self.at_detector.detect(image, estimate_tag_pose=True, camera_params=[self.fx, self.fy, img_width/8, img_height/8], tag_size=0.1)

        if len(tags) > 0:
            closest_tag = min(tags, key=lambda tag: self.calc_distance_away(tag))
            self.error = int(self.calc_rel_horizontal_angle(closest_tag, img_width))
            if (self.calc_distance_away(closest_tag) < 3):
                self.turn_lights_on(100)
                time.sleep(0.1)
                self.turn_lights_on(0)
        else:
            self.error = 0
        # saving image without color thsi time
        self.get_logger().info(f"\nNumber of tags: {len(tags)}")
        self.get_logger().info(f"\nDistance: {self.calc_distance_away(closest_tag)}")
        cv2.imwrite("image.png", image) 

    def calc_rel_horizontal_angle(self, tag, width):
        x = tag.center[0]
        rel_x = (x - width/2)
        return (np.degrees(np.arctan(rel_x/self.fx)))

    def calc_rel_vertical_angle(self, tag, height):
        y = tag.center[1]
        rel_y = (y - height/2)
        return (np.degrees(np.arctan(rel_y/self.fy)))

    def calc_distance_away(self, tag):
        return (np.linalg.norm(tag.pose_t))
    
    def turn_lights_on(self, level):
        """
        Turn the lights on.

        Args:
            level (int): The level to turn the lights on to. 0 is off, 100 is full
        """
        self.get_logger().info(f"Turning lights on to level {level}")
        commands = OverrideRCIn()
        commands.channels = [OverrideRCIn.CHAN_NOCHANGE] * 10
        commands.channels[8] = 1000 + level * 10
        commands.channels[9] = 1000 + level * 10
        self.command_pub.publish(commands)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetection()

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

