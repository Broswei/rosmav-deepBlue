import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from dt_apriltags import Detector
import numpy as np
from mavros_msgs.msg import OverrideRCIn

class AprilTagDetection(Node):

    fx = 1060.7
    fy = 1060.7
    desired_heading = 0

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

        self.lights_publisher = self.create_publisher(
            OverrideRCIn, "bluerov2/override_rc", self.override_callback, 10
        )


    def heading_callback(self, msg):
        update = Int16()
        update.data = msg.data + self.desired_heading
        self.desired_heading_publisher.publish(update)


    def image_callback(self, msg: Image):

        image = self.cvb.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite("image.png", image) 

        # Save the image

        # Get the dimensions of the image
        img_width = image.shape[1]
        img_height = image.shape[0]
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        tags = self.at_detector.detect(image, estimate_tag_pose=True, camera_params=[self.fx, self.fy, img_width/8, img_height/8], tag_size=0.1)
        color_img = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

        if tags:
            closest_tag = min(tags, key=lambda tag: self.calc_distance_away(tag))
            self.desired_heading = int(self.calc_rel_horizontal_angle(closest_tag, img_width))

            for tag in tags:
                for idx in range(len(tag.corners)):
                    cv2.line(color_img, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0), 2)

                cv2.putText(color_img, str(tag.tag_id),
                            org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=1.2,
                            color=(255, 0, 0))
                #self.get_logger().info(f"TAG ID: {tag.tag_id} \n X ANGLE: {self.calc_rel_horizontal_angle(tag, img_width)} \n Y ANGLE: {self.calc_rel_vertical_angle(tag,img_height)}\n DISTANCE: {self.calc_distance_away(tag)}")
        cv2.imwrite("image.png", image) 

    def override_callback(self, msg):
        pass

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

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int16
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# from dt_apriltags import Detector
# import numpy as np

# class AprilTagDetection(Node):

#     fx = 1060.7
#     fy = 1060.7
#     avg_rel_error = 0

#     def __init__(self):
#         super().__init__("april_tags")
        
#         self.cvb = CvBridge()

#         self.at_detector = Detector(families='tag36h11',
#                        nthreads=2,
#                        quad_decimate=1.0,
#                        quad_sigma=0.0,
#                        refine_edges=1,
#                        decode_sharpening=0.25,
#                        debug=0)

#         self.desired_heading_publisher = self.create_publisher(
#             Int16, "bluerov2/desired_heading", 10
#         )

#         self.heading_subscriber = self.create_subscription(
#             Int16, "bluerov2/heading", self.heading_callback, 10
#         )

#         self.subscription = self.create_subscription(
#             Image, "bluerov2/camera", self.image_callback, 10
#         )


#     def heading_callback(self, msg):
#         self.desired_heading_publisher.publish(msg.data + self.avg_rel_error)


#     def image_callback(self, msg: Image):

#         image = self.cvb.imgmsg_to_cv2(msg, "bgr8")

#         # Save the image
#         cv2.imwrite("image.png", image) 

#         # # Get the dimensions of the image
#         img_width = image.shape[0]
#         img_height = image.shape[1]
#         image = cv2.imread(image, cv2.IMREAD_GRAYSCALE)
        

#         tags = self.at_detector.detect(image, estimate_tag_pose=True, camera_params=[self.fx, self.fy, img_height/2, img_width/2], tag_size=0.1)
#         color_img = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
#         sum_rel_headings = 0
#         for tag in tags:
#             for idx in range(len(tag.corners)):
#                 cv2.line(color_img, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0), 2)

#             cv2.putText(color_img, str(tag.tag_id),
#                         org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
#                         fontFace=cv2.FONT_HERSHEY_SIMPLEX,
#                         fontScale=1.2,
#                         color=(255, 0, 0))
#             sum_rel_headings += self.calc_rel_horizontal_angle(tag, img_height)
#             print (f"TAG ID: {tag.tag_id} \n X ANGLE: {self.calc_rel_horizontal_angle(tag, img_height)} \n Y ANGLE: {self.calc_rel_vertical_angle(tag,img_width)}\n DISTANCE: {self.calc_distance_away(tag)}")

#         self.avg_rel_error = sum_rel_headings / len(tags)

#     def calc_rel_horizontal_angle(self, tag, height):
#         x = tag.center[0]
#         rel_x = (x - height/2)
#         return (np.degrees(np.arctan(rel_x/self.fx)))

#     def calc_rel_vertical_angle(self, tag, width):
#         y = tag.center[1]
#         rel_y = (y - width/2)
#         return (np.degrees(np.arctan(rel_y/self.fy)))

#     def calc_distance_away(self, tag):
#         return (np.linalg.norm(tag.pose_t))

# def main(args=None):
#     rclpy.init(args=args)
#     node = AprilTagDetection()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print("\nKeyboardInterrupt received, shutting down...")
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()

# if __name__=="__main__":
#     main()