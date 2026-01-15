import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import math

class SimpleTurtlesimKinematics(Node):
    def __init__(self):
        super().__init__('simple_turtlesim_kinematics')
        
        self.turtle1_pose_sub_ = self.create_subscription(Pose, '/turtle1/pose', self.turtle1_pose_callback, 10)
        self.turtle2_pose_sub_ = self.create_subscription(Pose, '/turtle2/pose', self.turtle2_pose_callback, 10)
        
        self.last_turtle1_pose_ = Pose()
        self.last_turtle2_pose_ = Pose()    

    def turtle1_pose_callback(self, msg):
        self.last_turtle1_pose_ = msg
    
    def turtle2_pose_callback(self, msg):
        self.last_turtle2_pose_ = msg

        Tx = self.last_turtle2_pose_.x - self.last_turtle1_pose_.x
        Ty = self.last_turtle2_pose_.y - self.last_turtle1_pose_.y
        Ttheta = self.last_turtle2_pose_.theta - self.last_turtle1_pose_.theta
        Theta_deg = Ttheta * 180 / math.pi

        self.get_logger().info("""
        Translation vector from turtle1 to turtle2: 
        Tx={0}, Ty={1}
        Rotation matrix from turtle1 to turtle2: 
        |R11   R12 | : {2} {3}
        |R21   R22 | : {4} {5}
        """.format(Tx, Ty, math.cos(Ttheta), math.sin(Ttheta), -math.sin(Ttheta), math.cos(Ttheta)))

def main():
    rclpy.init()
    node = SimpleTurtlesimKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()