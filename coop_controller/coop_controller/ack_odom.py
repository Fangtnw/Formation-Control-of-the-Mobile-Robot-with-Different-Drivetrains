#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from math import sin, cos, tan
import tf2_ros
from tf2_ros import TransformBroadcaster

wheel_distance = 0.55
length = 0.55

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.get_logger().info('Ackermann Odometry Node is running')

        self.left_encoder_ticks = 0
        self.right_encoder_ticks = 0
        self.left_vel = 0.0
        self.right_vel = 0.0
        self.angular_velocity = 0.0

        # Initialize the robot state variables
        self.Robot_X = 0.0
        self.Robot_Y = 0.0
        self.Robot_Yaw = 0.0
        self.Robot_LinVel = 0.0
        self.Robot_AngVel = 0.0

        self.odom_pub = self.create_publisher(Odometry, 'ack_odom', 100)
        # self.encoder_ticks_sub = self.create_subscription(
        #     Vector3Stamped, 'encoder_ticks', self.encoder_ticks_callback, 100)
        self.encoder_vel_sub = self.create_subscription(
            Vector3Stamped, 'ack_encoder_vel', self.encoder_vel_callback, 1)
        self.imu_sub = self.create_subscription(
            Vector3Stamped, 'ack_IMU', self.imu_callback, 1)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.publish_odometry)

    def encoder_ticks_callback(self, msg):
        self.left_encoder_ticks = msg.vector.x
        self.right_encoder_ticks = msg.vector.y

    def encoder_vel_callback(self, msg):
        self.left_vel = msg.vector.x
        self.right_vel = msg.vector.y
        self.steering_angle = msg.vector.z
        self.avg_vel = (self.left_vel+self.right_vel)/2

    def imu_callback(self, msg):
        # self.Robot_Yaw = msg.vector.z
        pass

    def publish_odometry(self):
        odom_msg = Odometry()

        # Set the frame_id field in the Odometry message
        odom_msg.header.frame_id = 'map'
        odom_msg.header.stamp = self.get_clock().now().to_msg()

        # Set the child_frame_id to "base_link"
        odom_msg.child_frame_id = "base_link"

        # Perform your odometry calculations here using encoder ticks, velocities, and IMU data
        # Replace the following placeholder values with your calculations
        time_step = 0.05  # replace with your actual time step
        self.forward_kinematic()
        self.odom_compute(time_step)

        odom_msg.pose.pose.position.x = self.Robot_X
        odom_msg.pose.pose.position.y = self.Robot_Y
        odom_msg.pose.pose.position.z = 0.0

        # Assuming IMU provides yaw directly
        odom_msg.pose.pose.orientation.z = sin(self.Robot_Yaw / 2)
        odom_msg.pose.pose.orientation.w = cos(self.Robot_Yaw / 2)

        odom_msg.twist.twist.linear.x = self.Robot_LinVel
        odom_msg.twist.twist.angular.z = self.Robot_AngVel

        # Set covariance matrices
        odom_msg.pose.covariance[0] = 0.1  # Example covariance value for x position
        odom_msg.pose.covariance[7] = 0.1  # Example covariance value for y position
        # ... Set other covariance values as needed

        odom_msg.twist.covariance[0] = 0.1  # Example covariance value for x linear velocity
        odom_msg.twist.covariance[35] = 0.1  # Example covariance value for z angular velocity
        # ... Set other covariance values as needed

        # Publish odometry message
        self.odom_pub.publish(odom_msg)

        # Publish TF transform
        transform = TransformStamped()
        transform.header = Header()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'ack_odom'
        transform.child_frame_id = 'ack_base_link'
        transform.transform.translation.x = self.Robot_X
        transform.transform.translation.y = self.Robot_Y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = sin(self.Robot_Yaw / 2)
        transform.transform.rotation.w = cos(self.Robot_Yaw / 2)
        self.tf_broadcaster.sendTransform(transform)

    def forward_kinematic(self):
        self.Robot_LinVel = self.avg_vel
        self.Robot_AngVel = self.avg_vel * tan(self.steering_angle) / length

    def odom_compute(self, time_step):
        temp_tetra = self.Robot_Yaw + (self.Robot_AngVel * time_step * 0.5)
        self.Robot_X = self.Robot_X + cos(temp_tetra) * self.Robot_LinVel * time_step
        self.Robot_Y = self.Robot_Y + sin(temp_tetra) * self.Robot_LinVel * time_step
        self.Robot_Yaw = self.Robot_Yaw + self.Robot_AngVel * time_step

def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    rclpy.spin(odometry_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
