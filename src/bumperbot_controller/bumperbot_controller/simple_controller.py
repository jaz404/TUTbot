#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from rclpy.time import Time
from rclpy.constants import S_TO_NS
import numpy as np
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')

        # Declare parameters
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.17)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value # double since wheel radius is a double
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value # also double

        self.get_logger().info(f" Using wheel radius: {self.wheel_radius_}") 
        self.get_logger().info(f" Using wheel separation: {self.wheel_separation_}") 

        # odom vars for calculation of lin and angular velocities (more details in odometry.md)
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()

        # helper vars for calculation of position and orientation (more details in odometry.md)
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, "bumperbot_controller/cmd_vel", self.vel_callback, 10)

        # create subscription to joint states
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.joint_callback, 10)

        # create odom publisher
        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom", 10)

        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2], 
        [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])

        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.position.x = 0.0
        self.odom_msg_.pose.pose.position.y = 0.0
        self.odom_msg_.pose.pose.position.z = 0.0
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0
        self.odom_msg_.twist.twist.linear.x = 0.0
        self.odom_msg_.twist.twist.linear.y = 0.0
        self.odom_msg_.twist.twist.linear.z = 0.0
        self.odom_msg_.twist.twist.angular.x = 0.0
        self.odom_msg_.twist.twist.angular.y = 0.0
        self.odom_msg_.twist.twist.angular.z = 0.0

        # create transform broadcaster
        self.tf_broadcaster_ = TransformBroadcaster(self)
        self.tf_msg_ = TransformStamped()
        self.tf_msg_.header.frame_id = "odom"
        self.tf_msg_.child_frame_id = "base_footprint"

        self.get_logger().info(f" The speed conversion matrix is: {self.speed_conversion_}")

    def vel_callback(self, msg):
        robot_speed = np.array([[msg.twist.linear.x], 
        [msg.twist.angular.z]])

        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1,0], wheel_speed[0,0]]

        self.wheel_cmd_pub_.publish(wheel_speed_msg)

    def joint_callback(self, msg):
        # Calculating the Rotational velocities of the wheels 
        # For more info about the formula used, check odometry.md
        dp_left = msg.position[1] - self.left_wheel_prev_pos_
        dp_right = msg.position[0] - self.right_wheel_prev_pos_

        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        dt_s = dt.nanoseconds / S_TO_NS

        if dt_s <= 0.0:
        # Nothing to integrate / differentiate yet; just update prevs and exit.
            self.prev_time_ = Time.from_msg(msg.header.stamp)
            self.left_wheel_prev_pos_ = msg.position[1]
            self.right_wheel_prev_pos_ = msg.position[0]
            return

        self.left_wheel_prev_pos_ = msg.position[1]
        self.right_wheel_prev_pos_ = msg.position[0]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        fi_left = dp_left/ (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right/ (dt.nanoseconds / S_TO_NS)

        # now we can calculate the linear and angular velocities of the robot
        linear = self.wheel_radius_ * (fi_left + fi_right)/2
        angular = self.wheel_radius_ * (fi_right- fi_left)/self.wheel_separation_

        d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_

        self.theta_ += d_theta
        self.x_ += d_s * np.cos(self.theta_)
        self.y_ += d_s * np.sin(self.theta_)

        ################ format everything for odom message ################
        # convert rot to euler
        q = quaternion_from_euler(0, 0, self.theta_) # convert euler to quaternion
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]

        # update time stamp
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()

        # set position
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.position.z = 0.0
        
        # set twist
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular

        # self.get_logger().info("Linear velocity: %f  Angular velocity: %f" % (linear, angular))
        # self.get_logger().info("X: %f  Y: %f  Theta: %f" % (self.x_, self.y_, self.theta_))

        self.tf_msg_.header.stamp = self.get_clock().now().to_msg()
        self.tf_msg_.transform.translation.x = self.x_
        self.tf_msg_.transform.translation.y = self.y_
        self.tf_msg_.transform.translation.z = 0.0
        self.tf_msg_.transform.rotation.x = q[0]
        self.tf_msg_.transform.rotation.y = q[1]
        self.tf_msg_.transform.rotation.z = q[2]
        self.tf_msg_.transform.rotation.w = q[3]

        # publish the odom message
        self.odom_pub_.publish(self.odom_msg_)
        
        # publish the transform
        self.tf_broadcaster_.sendTransform(self.tf_msg_)

def main():
    rclpy.init()
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()