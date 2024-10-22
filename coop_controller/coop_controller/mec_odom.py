#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from math import sin, cos
import tf2_ros
from tf2_ros import TransformBroadcaster
import tf_transformations

# wheel_distance = 0.5
radius = 0.076;                   
wheelbase = 0.35;               

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.get_logger().info('Mecanum Odometry Node is running')

        self.left_encoder_ticks = 0
        self.right_encoder_ticks = 0
        self.left_vel = 0.0
        self.motor1_vel = 0.0
        self.motor2_vel = 0.0
        self.motor3_vel = 0.0
        self.motor4_vel = 0.0
        self.right_vel = 0.0
        self.angular_velocity = 0.0

        # Initialize the robot state variables
        self.Robot_X = 0.0
        self.Robot_Y = 0.0
        self.Robot_Roll = 0.0
        self.Robot_Pitch = 0.0
        self.Robot_Yaw = 0.0
        self.Robot_LinVel = 0.0
        self.Robot_AngVel = 0.0
        self.odom_pub = self.create_publisher(Odometry, 'mec_odom_raw', 100)
        # self.encoder_ticks_sub = self.create_subscription(
        #     Vector3Stamped, 'diff_encoder_ticks', self.encoder_ticks_callback, 100)
        self.encoder_vel_sub = self.create_subscription(
            Twist, '/mec_encoder_vel', self.encoder_vel_callback, 1)
        # self.imu_sub = self.create_subscription(
        #     Vector3Stamped, 'diff_imu', self.imu_callback, 1)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_odometry)

    def encoder_ticks_callback(self, msg):
        self.motor1_ticks = msg.linear.x
        self.motor2_ticks = msg.linear.y
        self.motor3_ticks = msg.angular.x
        self.motor4_ticks = msg.angular.y
        

    def encoder_vel_callback(self, msg):
        self.motor1_vel = msg.linear.x
        self.motor2_vel = msg.linear.y
        self.motor3_vel = msg.angular.x
        self.motor4_vel = msg.angular.y
        
    # def imu_callback(self, msg):
    #     # self.Robot_Roll = msg.vector.x
    #     # self.Robot_Pitch = msg.vector.y
    #     # self.Robot_Yaw = msg.vector.z
    #     pass
        
    def publish_odometry(self):
        odom_msg = Odometry()

        # Set the frame_id field in the Odometry message
        odom_msg.header.frame_id = 'mec_odom_raw'
        odom_msg.header.stamp = self.get_clock().now().to_msg()

        # Set the child_frame_id to "base_link"
        odom_msg.child_frame_id = "base_footprint_mec"

        # Perform your odometry calculations here using encoder ticks, velocities, and IMU data
        # Replace the following placeholder values with your calculations
        time_step = 0.1
        # self.forward_kinematic()
        self.odom_compute(time_step)

        odom_msg.pose.pose.position.x = self.Robot_X
        odom_msg.pose.pose.position.y = self.Robot_Y
        odom_msg.pose.pose.position.z = 0.0

        # Assuming IMU provides yaw directly
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.Robot_Yaw)
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        odom_msg.twist.twist.linear.x = self.Robot_LinVel
        odom_msg.twist.twist.angular.z = self.Robot_AngVel

        # Set covariance matrices
        # odom_msg.pose.covariance[0] = 0.1  # Example covariance value for x position
        # odom_msg.pose.covariance[7] = 0.1  # Example covariance value for y position
        # # ... Set other covariance values as needed

        # odom_msg.twist.covariance[0] = 0.1  # Example covariance value for x linear velocity
        # odom_msg.twist.covariance[35] = 0.1  # Example covariance value for z angular velocity
        # ... Set other covariance values as needed

        # Publish odometry message
        self.odom_pub.publish(odom_msg)

        map_to_odom_transform = TransformStamped()
        map_to_odom_transform.header.stamp = self.get_clock().now().to_msg()
        map_to_odom_transform.header.frame_id = 'map'
        map_to_odom_transform.child_frame_id = 'odom_mec'
        # map_to_odom_transform.transform.translation.x = 1.0  # Adjust as needed
        # map_to_odom_transform.transform.translation.y = 0.0  # Adjust as needed
        # map_to_odom_transform.transform.translation.z = 0.0  # Adjust as needed
        # map_to_odom_transform.transform.rotation.z = 0.0  # Adjust as needed
        # map_to_odom_transform.transform.rotation.w = 1.0  # Adjust as needed
        # self.tf_broadcaster.sendTransform(map_to_odom_transform)
        
        # Publish TF transform
        transform = TransformStamped()
        transform.header = Header()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom_mec'
        transform.child_frame_id = 'base_footprint_mec'
        transform.transform.translation.x = self.Robot_X
        transform.transform.translation.y = self.Robot_Y
        transform.transform.translation.z = 0.0
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.Robot_Yaw)
        transform.transform.rotation.z = sin(self.Robot_Yaw / 2)
        transform.transform.rotation.w = cos(self.Robot_Yaw / 2)

        # transform.transform.rotation.x = quaternion[0]
        # transform.transform.rotation.y = quaternion[1]
        # transform.transform.rotation.z = quaternion[2]
        # transform.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(transform)

        # Publish TF transform from 'base_link' to 'base_footprint'
        transform_base_link_to_base_footprint = TransformStamped()
        transform_base_link_to_base_footprint.header.stamp = self.get_clock().now().to_msg()
        transform_base_link_to_base_footprint.header.frame_id = 'base_link_follower'
        transform_base_link_to_base_footprint.child_frame_id = 'base_footprint_mec'
        transform_base_link_to_base_footprint.transform.translation.x = 0.0  # Adjust as needed
        transform_base_link_to_base_footprint.transform.translation.y = 0.0  # Adjust as needed
        transform_base_link_to_base_footprint.transform.translation.z = 0.0  # Adjust as needed
        transform_base_link_to_base_footprint.transform.rotation.z = 0.0  # Adjust as needed
        transform_base_link_to_base_footprint.transform.rotation.w = 1.0  # Adjust as needed
        # self.tf_broadcaster.sendTransform(transform_base_link_to_base_footprint)

    def forward_kinematic(self):
        # self.Robot_LinVel = (self.motor1_vel + self.motor2_vel + self.motor3_vel + self.motor4_vel) * radius / 4 
        # self.Robot_AngVel = (self.motor4_vel - self.motor3_vel - self.motor2_vel + self.motor1_vel) * radius /  ( 4 * wheelbase)
        self.Robot_LinVel = (self.motor1_vel + self.motor2_vel + self.motor3_vel + self.motor4_vel)  / 4

        # Calculate angular velocity
        self.Robot_AngVel = ((self.motor4_vel - self.motor3_vel - self.motor2_vel + self.motor1_vel)  ) / (4 * wheelbase)


    def odom_compute(self, time_step):
        # theta = self.Robot_Yaw + (self.Robot_AngVel * time_step * 0.5)
        # self.Robot_X = self.Robot_X + (cos(theta) * self.Robot_LinVel * time_step)
        # self.Robot_Y = self.Robot_Y + (sin(theta) * self.Robot_LinVel * time_step)
        # self.Robot_Yaw = self.Robot_Yaw + self.Robot_AngVel * time_step

        self.Robot_X = self.Robot_X + (((self.motor1_vel + self.motor2_vel + self.motor3_vel + self.motor4_vel)  / 4 ) * time_step)
        self.Robot_Y = self.Robot_Y + (((-self.motor1_vel + self.motor2_vel + self.motor3_vel - self.motor4_vel)  / 4 ) * time_step)
        self.Robot_Yaw = self.Robot_Yaw+ (((-self.motor1_vel + self.motor2_vel - self.motor3_vel + self.motor4_vel)  /  ( 4 * wheelbase) ) * time_step)

def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    rclpy.spin(odometry_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
