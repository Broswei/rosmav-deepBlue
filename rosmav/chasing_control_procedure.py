import rclpy
from rclpy.node import Node
import apriltag_detection as atd
import lane_following as lf

from mavros_msgs.msg import ManualControl

class CCPNode (Node):

    def __init__(self):
        super().__init__("CCP")

        self.offensive_mode = False
        self.numTags = 0
        self.numLines = 0

        self.desired_depth_pub = self.create_publisher(
            ManualControl, "bluerov2/manual_control", 10
        )

    def forwards(self): 
        pass

    def backwards(self):
        pass
        
    def patrol_the_sea(self):
        pass

    def offensive(self):
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