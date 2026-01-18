import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class SimpleTfKinematics(Node):
    def __init__(self):
        super().__init__('simple_tf_kinematics')

        self.static_tf_broadcaster_ = StaticTransformBroadcaster(self)
        self.dynamic_tf_broadcaster_ = TransformBroadcaster(self)

        # it accepts a TransformStamped message 
        self.static_transform_stamped_ = TransformStamped()
        self.dynamic_transform_stamped_ = TransformStamped()

        # helpers for dynamic transforms [this is only for demo to make the transform move linearly in x-direction]
        self.x_increment_ = 0.01
        self.last_x_ = 0.0

        self.static_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.static_transform_stamped_.header.frame_id = "bumperbot_base"
        self.static_transform_stamped_.child_frame_id = "bumperbot_top"

        # Definfing the transformation from base to top frame
        # (i) making the bumperbot top frame 0.3m above the base frame
        self.static_transform_stamped_.transform.translation.x = 0.0
        self.static_transform_stamped_.transform.translation.y = 0.0
        self.static_transform_stamped_.transform.translation.z = 0.3
        # (ii) defining the rotation
        self.static_transform_stamped_.transform.rotation.x = 0.0
        self.static_transform_stamped_.transform.rotation.y = 0.0
        self.static_transform_stamped_.transform.rotation.z = 0.0
        self.static_transform_stamped_.transform.rotation.w = 1.0

        self.static_tf_broadcaster_.sendTransform(self.static_transform_stamped_)

        self.get_logger().info("Publishing static transform between %s and %s frames" % (self.static_transform_stamped_.header.frame_id, self.static_transform_stamped_.child_frame_id))

        self.timer_ = self.create_timer(0.1, self.timer_callback) # We will be updating the dynamic transform at 10 Hz

    '''This callback is called at 10 Hz for the dynamic transform publishing'''
    def timer_callback(self):
        # dynamic transform from odom to base frame
        self.dynamic_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_transform_stamped_.header.frame_id = "odom"
        self.dynamic_transform_stamped_.child_frame_id = "bumperbot_base"
        
        # transformation
        self.dynamic_transform_stamped_.transform.translation.x = self.last_x_ + self.x_increment_
        self.dynamic_transform_stamped_.transform.translation.y = 0.0
        self.dynamic_transform_stamped_.transform.translation.z = 0.0

        self.dynamic_transform_stamped_.transform.rotation.x = 0.0
        self.dynamic_transform_stamped_.transform.rotation.y = 0.0
        self.dynamic_transform_stamped_.transform.rotation.z = 0.0
        self.dynamic_transform_stamped_.transform.rotation.w = 1.0

        self.dynamic_tf_broadcaster_.sendTransform(self.dynamic_transform_stamped_)

        self.get_logger().info("Publishing dynamic transform between %s and %s frames" % (self.dynamic_transform_stamped_.header.frame_id, self.dynamic_transform_stamped_.child_frame_id))

        self.last_x_ += self.x_increment_

        

def main():
    rclpy.init()
    simple_tf_kinematics = SimpleTfKinematics()
    rclpy.spin(simple_tf_kinematics)
    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()