#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, TransformStamped , Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from math import sin, cos, tan
import tf2_ros
from tf2_ros import TransformBroadcaster
import math

WHEEL_DISTANCE = 0.5
LENGTH = 0.5
IMU_WEIGHT = 0.6  # Relative weight for IMU
ODOMETRY_WEIGHT = 0.4  # Relative weight for odometry
TIME_STEP = 0.1  # Time step for the update rate

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.get_logger().info('Ackermann Odometry Node is running')

        self.left_encoder_ticks = 0
        self.right_encoder_ticks = 0
        self.left_vel = 0.0
        self.right_vel = 0.0
        self.angular_velocity = 0.0
        self.avg_vel = 0.0
        self.steering_angle = 0.0
        self.turn = 0.0

        # Initialize the robot state variables
        self.Robot_X = 0.0
        self.Robot_Y = 0.0
        self.Robot_Yaw = 0.0
        self.Robot_LinVel = 0.0
        self.Robot_AngVel = 0.0

        self.odom_pub = self.create_publisher(Odometry, 'ack_odom_raw', 100)
        # self.encoder_ticks_sub = self.create_subscription(
        #     Vector3Stamped, 'encoder_ticks', self.encoder_ticks_callback, 100)
        self.encoder_vel_sub = self.create_subscription(
            Twist, 'ack_encoder_vel', self.encoder_vel_callback, 1)
        self.imu_sub = self.create_subscription(
            Imu, 'ack_imu_raw', self.imu_callback, 1)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_odometry)

    def encoder_ticks_callback(self, msg):
        self.left_encoder_ticks = msg.linear.y
        self.right_encoder_ticks = msg.linear.x

    def encoder_vel_callback(self, msg):
        self.left_vel = -msg.linear.y
        self.right_vel =  -msg.linear.x
        self.steering_angle = -msg.angular.y
        #self.steering_angle = 0.0
        #self.avg_vel = (self.left_vel + self.right_vel)/ 2

    def imu_callback(self, msg):
        # self.Robot_Yaw = msg.vector.z
        self.Robot_AngVel_imu = msg.angular_velocity.z*0.017453  #degree/s to rad/s
        pass

    def publish_odometry(self):
        
        self.forward_kinematic()
        self.fuse_angular_velocity()
        self.odom_compute()

        odom_msg = Odometry()

        # Set the frame_id field in the Odometry message
        odom_msg.header.frame_id = 'odom'
        odom_msg.header.stamp = self.get_clock().now().to_msg()

        # Set the child_frame_id to "base_link"
        odom_msg.child_frame_id = "base_footprint"

        # Perform your odometry calculations here using encoder ticks, velocities, and IMU data
        # Replace the following placeholder values with your calculations

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
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.Robot_X
        transform.transform.translation.y = self.Robot_Y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = sin(self.Robot_Yaw / 2)
        transform.transform.rotation.w = cos(self.Robot_Yaw / 2)
        self.tf_broadcaster.sendTransform(transform)

    def fuse_angular_velocity(self):
        # Fused angular velocity using weighted average
        wheel_ang_vel = self.Robot_AngVel_wheel
        imu_ang_vel = self.Robot_AngVel_imu
        # Combine angular velocities
        self.Robot_AngVel = (IMU_WEIGHT * imu_ang_vel) + (ODOMETRY_WEIGHT * wheel_ang_vel)


    def forward_kinematic(self):
        self.Robot_LinVel = (self.left_vel + self.right_vel)* 0.5
        self.Robot_AngVel_wheel = (self.right_vel + self.left_vel)*0.5* tan(math.radians(self.steering_angle)) / WHEEL_DISTANCE

    def odom_compute(self):
        temp_thetha = self.Robot_Yaw + (self.Robot_AngVel * TIME_STEP * 0.5)
        self.Robot_X += cos(temp_thetha) * self.Robot_LinVel * TIME_STEP
        self.Robot_Y += sin(temp_thetha) * self.Robot_LinVel * TIME_STEP
        self.Robot_Yaw += self.Robot_AngVel * TIME_STEP

def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    rclpy.spin(odometry_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
