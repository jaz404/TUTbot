import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class SimpleTrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('simple_trajectory_plotter')

        # create param topic to which we want to publish the trajectory (default is /bumperbot_controller/trajectory)
        # now this can be changed during runtime: ros2 run bumperbot_utils simple_trajectory_plotter --ros-args -p odom_topic:=/your_topic

        self.declare_parameter('odom_topic', '/bumperbot_controller/odom')
        self.declare_parameter('trajectory_topic', '/bumperbot_controller/trajectory')

        # Subscriber to the /bumperbot_controller/odom topic
        self.odom_subscriber_ = self.create_subscription(Odometry, self.get_parameter('odom_topic').get_parameter_value().string_value, self.odom_callback, 10)

        # create pub to /bumperbot_controller/trajectory
        self.trajectory_publisher_ = self.create_publisher(Path, self.get_parameter('trajectory_topic').get_parameter_value().string_value, 10)

        self.traj_msg_ = Path()
        self.traj_msg_.header.frame_id = "odom"
        
    def odom_callback(self, msg):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.traj_msg_.poses.append(pose_stamped)
        self.trajectory_publisher_.publish(self.traj_msg_)

def main():
    rclpy.init()
    node = SimpleTrajectoryPlotter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
