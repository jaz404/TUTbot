import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from bumperbot_msgs.srv import GetTransform
from tf_transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse

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

        self.rotations_counter_ = 0
        self.last_orientation_ = quaternion_from_euler(0.0, 0.0, 0.0)
        self.orientation_increment_ =  quaternion_from_euler(0.0, 0.0, 0.05) # yaw will increment whenever the timer expires

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

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

        # service server
        self.get_transform_srv_ = self.create_service(GetTransform, "get_transform", self.get_transform_callback)

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

        # calculate the new orientation
        q = quaternion_multiply(self.last_orientation_, self.orientation_increment_)

        self.dynamic_transform_stamped_.transform.rotation.x = q[0]
        self.dynamic_transform_stamped_.transform.rotation.y = q[1]
        self.dynamic_transform_stamped_.transform.rotation.z = q[2]
        self.dynamic_transform_stamped_.transform.rotation.w = q[3]

        self.dynamic_tf_broadcaster_.sendTransform(self.dynamic_transform_stamped_)

        self.get_logger().info("Publishing dynamic transform between %s and %s frames" % (self.dynamic_transform_stamped_.header.frame_id, self.dynamic_transform_stamped_.child_frame_id))

        # set the last x and orientation
        self.last_x_ += self.x_increment_

        # set the rotations counter 
        self.rotations_counter_ += 1
        
        self.last_orientation_ = q

        # if rotations counter is 10, reverse the x increment
        if self.rotations_counter_ >= 100:
            self.orientation_increment_ = quaternion_inverse(self.orientation_increment_)
            self.rotations_counter_ = 0


    def get_transform_callback(self, request, response):
        self.get_logger().info("requested transform between %s and %s frames" % (request.frame_id, request.child_frame_id))
        
        requested_transform = TransformStamped()
        try:
            # using the lookup_transform, we can get the transform between the two frames
            requested_transform = self.tf_buffer_.lookup_transform(request.frame_id, request.child_frame_id, rclpy.time.Time())
        except TransformException:
            self.get_logger().error("An error occurred while transforming %s and %s"% (request.frame_id, request.child_frame_id))
            response.success = False
            return response

        response.success = True
        response.transform = requested_transform


def main():
    rclpy.init()
    simple_tf_kinematics = SimpleTfKinematics()
    rclpy.spin(simple_tf_kinematics)
    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()