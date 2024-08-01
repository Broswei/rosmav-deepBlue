import rclpy
from rclpy.node import Node

from mavros_msgs.msg import ManualControl, OverrideRCIn
from sensor_msgs.msg import Image
from std_msgs.msg import Int16

import cv2
from cv_bridge import CvBridge
from dt_apriltags import Detector
import numpy as np
import time

import rosmav.lane_detection as ld

class CCPNode (Node):

    fx = 1060.7
    fy = 1060.7
    desired_heading = 0
    curr_heading = 0

    def __init__(self):
        super().__init__("CCP")

        self.going_forward = True
        self.april_mode = False
        self.at_detector = Detector()
        self.cvb = CvBridge()

        self.manual_control_pub = self.create_publisher(
            ManualControl, "bluerov2/manual_control", 10
        )

        self.override_pub = self.create_publisher(
            OverrideRCIn, "bluerov2/override_rc", 10
        )

        self.desired_heading_pub = self.create_publisher(
            Int16, "bluerov2/desired_heading", 10
        )

        self.image_sub = self.create_subscription(
            Image, "bluerov2/camera", self.image_callback, 10
        )

        self.heading_sub = self.create_subscription(
            Int16, "bluerov2/heading", self.heading_callback, 10
        )

        self.create_timer(8, self.switch_dir)


    # Case by case image callbacks
    def image_callback(self, msg):


        image = self.cvb.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite("image.png", image) 

        img_width = image.shape[1]
        img_height = image.shape[0]
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        tags = self.at_detector.detect(image, estimate_tag_pose=True, camera_params=[self.fx, self.fy, img_width/8, img_height/8], tag_size=0.1)
    
        if len(tags) > 0: # april tag
            temp_stop = ManualControl()
            temp_stop.x = 0.0
            self.manual_control_pub.publish(temp_stop)
            closest_tag = min(tags, key=lambda tag: self.calc_distance_away(tag))
            self.desired_heading = int(self.calc_rel_horizontal_angle(closest_tag, img_width))
            self.april_mode = True
        else: # lane following
            self.april_mode = False
            image = self.cvb.imgmsg_to_cv2(msg, "bgr8")
            image, lines = ld.detect_lines(image, 300, 350, 5, 100, 150)
            image = ld.draw_lines(image, lines)
            slopes, intercepts = ld.get_slopes_intercepts(lines)
            cv2.imwrite("image.png", image) 
            if len(slopes) != 0:
                center_slope = max(abs(slope) for slope in slopes)

                self.desired_heading = (90 - np.degrees(np.arctan(center_slope))) + self.curr_heading
            else:
                print("No April Tags or lines to follow have been found. Rotating 178 deg.")
                self.desired_heading = self.curr_heading + 178

        new_desired_heading = Int16()
        
        new_desired_heading.data = int(self.desired_heading)

        self.desired_heading_pub.publish(new_desired_heading)

        if self.april_mode:
            send_mc = ManualControl()
            send_override = OverrideRCIn()
            if self.calc_distance_away(closest_tag) > 4:
                send_mc.x = 10.0
                self.manual_control_pub.publish(send_mc)
            else:
                send_mc.x = 0.0
                self.manual_control_pub.publish(send_mc)
                send_override.channels[8] = 2000
                send_override.channels[9] = 2000
                self.override_pub.publish(send_override)
                time.sleep(1000)
                send_override.channels[8] = 1000
                send_override.channels[9] = 1000
                self.override_pub.publish(send_override)
        else:
            self.patrol_the_sea()


    def heading_callback(self, msg):
        
        self.curr_heading = msg.data
    

    # April Tag Functions
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


    # Patrolling the seas
    def forwards(self, msg): 
        msg.x = 20.0

    def backwards(self, msg):
        msg.x = -20.0
        
    def patrol_the_sea(self):
        """
        Go forwards if going forwards is true and goes backwards otherwise.
        """
        msg = ManualControl()
        if self.going_forward:
            self.forwards(msg)
        else:
            self.backwards(msg)
        self.manual_control_pub.publish(msg)
    
    
    # Timer handling
    def switch_dir(self):
        """
        Doesn't do shit
        """
        if self.going_forward and not self.april_mode:
            self.going_forward = False
        elif not self.going_forward and not self.april_mode:
            self.going_forward = True
        else:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CCPNode()

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