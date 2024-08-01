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

        self.command_pub = self.create_publisher(
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
        time.sleep(0.5) # time delay

        image = msg
        cv2.imwrite("image.png", image) 

        img_width = image.shape[1]
        img_height = image.shape[0]
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        tags = self.at_detector.detect(image, estimate_tag_pose=True, camera_params=[self.fx, self.fy, img_width/8, img_height/8], tag_size=0.1)
    
        if len(tags) > 0: # april tag   sign
            temp_stop = ManualControl()
            temp_stop.x = 0.0
            temp_stop.y = 0.0
            self.manual_control_pub.publish(temp_stop)
            closest_tag = min(tags, key=lambda tag: self.calc_distance_away(tag))

            # self.desired_heading = int(self.calc_rel_horizontal_angle(closest_tag, img_width)) + self.curr_heading
            self.desired_heading = self.curr_heading - int(self.calc_rel_horizontal_angle(closest_tag, img_width)) 
            self.desired_heading = (self.desired_heading + 360) % 360
            # might be incorrect


            self.april_mode = True
        else: # lane following
            self.april_mode = False
            image = self.cvb.imgmsg_to_cv2(msg, "bgr8")
            image, lines = ld.detect_lines(image, 300, 350, 5, 100, 150)
            image = ld.draw_lines(image, lines)
            slopes, intercepts = ld.get_slopes_intercepts(lines)
            cv2.imwrite("image.png", image) 
            if len(slopes) != 0:
                center_slope, index = max((abs(slope), i) for slope, i in slopes)

                if slopes[index] >= 0: 
                    self.desired_heading = (self.curr_heading - (90 - np.degrees(np.arctan(center_slope))))
                else:
                    self.desired_heading = (self.curr_heading + (90 + np.degrees(np.arctan(center_slope))))
            else:
                self.desired_heading = self.curr_heading

        new_desired_heading = Int16()
        
        new_desired_heading.data = int(self.desired_heading)

        self.desired_heading_pub.publish(new_desired_heading)

        if self.april_mode:
            send_mc = ManualControl()
            if self.calc_distance_away(closest_tag) > 1:
                send_mc.x = 10.0
                self.manual_control_pub.publish(send_mc)
            else:
                send_mc.x = 0.0
                send_mc.y = 0.0
                self.manual_control_pub.publish(send_mc)
                self.turn_lights_on(100)
                time.sleep(0.1)
                self.turn_lights_on(0)
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

    # Patrolling the seas
    def forward(self, msg): 
        msg.x = 20.0

    def back(self, msg):
        msg.x = -20.0
        
    def patrol_the_sea(self):
        """
        Go forwards if going forwards is true and goes backwards otherwise.
        """
        msg = ManualControl()
        if self.going_forward:
            self.forward(msg)
        else:
            self.back(msg)
        self.manual_control_pub.publish(msg)
    
    
    # Timer handling
    def switch_dir(self):
       self.going_forward = not self.going_forward

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