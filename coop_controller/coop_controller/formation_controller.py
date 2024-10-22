import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_vector3
from math import cos , sin, pi
import argparse
import numpy as np
import math
import tf_transformations

max_vx = 0.35
max_vy = 0.35
max_rz= 0.5
PALLET_ALLOWANCE = 0.25

class FormationController(Node):
    def __init__(self,follower_type):
        super().__init__('formation_controller')
        self.follower_type = follower_type
        if self.follower_type == 'mecanum':
            self.robot1_frame = 'base_footprint'
            self.robot2_frame = 'base_footprint_mec'
        elif self.follower_type == 'diffdrive':
            self.robot1_frame = 'base_footprint'
            self.robot2_frame = 'base_footprint_diff'
        elif self.follower_type == 'aruco':
            self.robot1_frame = 'aruco_frame'
            self.robot2_frame = 'camera_frame'
        self.leader_x = 0.0
        self.leader_w = 0.0
        self.direction = 1.0
        
        self.previous_yaw = 0
        self.previous_ty = 0
        self.kp_x = 1
        self.ki_x = 0.0  # Initialize for integral term
        self.kp_y = 0.2  # New gain for y-direction
        self.ki_y = 0.0  # Initialize for integral term
        self.kp_yaw = 1
        self.ki_yaw = 0  # Initialize for integral term

        self.kl_x = 1  #8.75
        self.kl_yw = 20    #38
        self.kl_yaw = 0

        # Variables to store integral errors for PI control
        self.integral_error_x = 0
        self.integral_error_y = 0  # New variable for y-direction error
        self.integral_error_yaw = 0
        
        self.follower_type = follower_type

        self.publisher = self.create_publisher(Twist, 'cmd_vel_follower', 10)
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        #subscribe for leader Twist command
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_pure',
            self.cmd_vel_callback,
            10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.control_callback)

    def cmd_vel_callback(self, msg):
        self.leader_x = msg.linear.x 
        self.leader_w = msg.angular.z 
        # pass

    def control_callback(self):
        try:
            cmd_msg = Twist()
            transform = self.tf_buffer.lookup_transform(
                self.robot1_frame, self.robot2_frame, rclpy.time.Time())
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z
            x = transform.transform.rotation.x
            y = transform.transform.rotation.y
            w = transform.transform.rotation.w
            z = transform.transform.rotation.z
            euler = tf_transformations.euler_from_quaternion([x, y, w, z])
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]
            leader_cmd_msg = Twist()

            self.get_logger().info(
                f"Translation: x:{tx:.2f}, y:{ty:.2f}, z:{tz:.2f} , "
                f"Rotation: roll:{roll:.2f}, pitch:{pitch:.2f}, yaw:{yaw:.4f}" 
            )

            # if tx > 1.65:
            #     self.leader_x = 0.0
            #     self.leader_w = 0.0
            #     self.get_logger().warn("waiting for follower")

            #mecanum_follower case
            if self.follower_type == 'mecanum':
                error_x = (tx - 1.65)
                self.integral_error_x += error_x  # Accumulate error for integral term
                linear_vel_x = (self.kp_x * error_x) + (-self.leader_x * self.kl_x) 
                error_y = ty
                self.integral_error_y += error_y  # Accumulate error for integral term
                linear_vel_y = (self.kp_y * error_y) + (-self.leader_w * self.kl_yw)

                linear_vel_x = max(min(linear_vel_x, max_vx), -max_vx)
                linear_vel_y = max(min(linear_vel_y, max_vx), -max_vx)

                if abs(error_x) < 0.2 and abs(error_y) < 0.2:
                    # Adjust yaw with damping, considering previous direction:
                    # if self.direction == 1.0 and abs(yaw) < abs(self.previous_yaw):   # Check turn direction
                    #     self.direction = -1.0 
                    # elif self.direction == -1.0 and abs(yaw) < abs(self.previous_yaw):   # Check turn direction
                    #     self.direction = 1.0
                    error_yaw = yaw
                    self.integral_error_yaw += error_yaw  # Accumulate error for integral term
                    angular_vel = ((self.kp_yaw * error_yaw ) + (self.ki_yaw * self.integral_error_yaw))  + (self.leader_w * self.kl_yaw) # Apply direction factor for yaw
                else:
                    angular_vel = 0.0  # Prioritize x, y if they are large
                cmd_msg.linear.y = linear_vel_y
                angular_vel = max(min(angular_vel, max_rz), -max_rz)

            #differential_drive robot follower case
            elif self.follower_type == 'diffdrive':
                error_x = (tx - 1.60)
                error_y = ty
                error_yaw = yaw

                self.integral_error_x += error_x  # Accumulate error for integral term
                self.integral_error_yaw += error_yaw  # Accumulate error for integral term
                
                linear_vel_y = 0.0
                #try to decrease ty
                if abs(error_y) > 0.1:
                    # angular_vel = -np.sign(self.kp_yaw * error_yaw)*((self.leader_w * self.kl_yaw)+(self.kp_y * error_y))
                    linear_vel_x = (-self.leader_x * self.kl_x) + (self.kp_y * (-np.sign(error_x+0.3))*(error_y))
                    if abs(error_yaw) < 0.25:
                        angular_vel = ((np.sign(error_y))*self.kp_y * error_y)
                    else:
                        angular_vel = 0.0
                else:
                    linear_vel_x = self.kp_x * error_x
                    angular_vel = self.kp_yaw * error_yaw
                
                  # Apply direction factor for yaw
                linear_vel_x = max(min(linear_vel_x, max_vx), -max_vx)
                angular_vel = max(min(angular_vel, max_rz), -max_rz)

            elif self.follower_type == 'aruco':
                error_x = (tx - 1.45)
                self.integral_error_x += error_x  # Accumulate error for integral term
                # linear_vel_x = self.kp_x * error_x + self.ki_x * self.integral_error_x
                error_y = ty
                self.integral_error_y += error_y  # Accumulate error for integral term
                error_yaw = yaw 
                self.integral_error_yaw += error_yaw
                # linear_vel_y = self.kp_y * error_y + self.ki_y * self.integral_error_y
                
                if self.leader_w != 0 and self.leader_x > 0:
                    # linear_vel_x = (self.kp_x * error_x) + (-self.leader_x * self.kl_x) - abs(self.leader_w * self.kl_yw)
                    linear_vel_x = (self.kp_x * error_x) + (self.ki_x * self.integral_error_x) + (-self.leader_x * self.kl_x) - abs(self.leader_w * self.kl_yw)
                    linear_vel_y = (self.kp_y * error_y) + (self.ki_y * self.integral_error_y) + (-self.leader_w * self.kl_yw)
                elif self.leader_w != 0 and self.leader_x < 0 :
                    # linear_vel_x = (self.kp_x * error_x) + (-self.leader_x * self.kl_x) + abs(self.leader_w * self.kl_yw)``
                    linear_vel_x = (self.kp_x * error_x) + (self.ki_x * self.integral_error_x)  + (-self.leader_x * self.kl_x) + abs(self.leader_w * self.kl_yw)
                    linear_vel_y = (self.kp_y * error_y) + (self.ki_y * self.integral_error_y) + (-self.leader_w * self.kl_yw)
                else:
                    linear_vel_x = (self.kp_x * error_x) + (self.ki_x * self.integral_error_x) + (-self.leader_x * self.kl_x) 
                    linear_vel_y = (self.kp_y * error_y) + (self.ki_y * self.integral_error_y) + (-self.leader_w * self.kl_yw)

                if abs(error_x) > PALLET_ALLOWANCE:
                    leader_cmd_msg.linear.x = 0.0
                    leader_cmd_msg.angular.z = 0.0
                    self.get_logger().warn("waiting for follower")
                else:
                    leader_cmd_msg.linear.x = self.leader_x
                    leader_cmd_msg.angular.z = self.leader_w      

                # if linear_vel_x <= 0.02 and linear_vel_y <= 0.02:
                #     linear_vel_x = 0.0
                #     linear_vel_y = 0.0

                # if abs(error_x) < 0.2 and abs(error_y) < 0.2:
                #       # Accumulate error for integral term
                #     # angular_vel = ((self.kp_yaw * error_yaw ) + (self.ki_yaw * self.integral_error_yaw))  # Apply direction factor for yaw
                #     angular_vel = ((self.kp_yaw * error_yaw ) + (self.ki_yaw * self.integral_error_yaw))  + (self.leader_w * self.kl_yaw) 
                #     # angular_vel = 0.0
                # else:
                angular_vel = 0.0  # Prioritize x, y if they are large
                # cmd_msg.linear.y = linear_vel_y
                
                linear_vel_x = max(min(linear_vel_x, max_vx), -max_vx)
                linear_vel_y = max(min(linear_vel_y, max_vy), -max_vy)
                angular_vel = max(min(angular_vel, max_rz), -max_rz)
        
            else:
                self.get_logger().error(f"Invalid follower type '{self.follower_type}'")
                exit()



            # self.get_logger().info(
            #     f"Follower_command: x:{linear_vel_x:.2f}, z:{angular_vel:.2f} "
            # )
            
            self.previous_ty = ty 
            self.previous_yaw = yaw 
            if abs(linear_vel_x) <= 0.05 and abs(linear_vel_y) <= 0.05:
                cmd_msg.linear.x = 0.0
                cmd_msg.linear.y = 0.0
                cmd_msg.angular.z = 0.0
            else:
                cmd_msg.linear.x = linear_vel_x
                cmd_msg.linear.y = linear_vel_y
                cmd_msg.angular.z = angular_vel
            
            self.publisher.publish(cmd_msg)
            self.publisher_cmd_vel.publish(leader_cmd_msg)

        except Exception as e:
            self.get_logger().error(f"Error looking up transform: {str(e)}")
            return

        # linear_vel = self.leader_x * cos(yaw*180/pi) + (k_x * ((tx-1) - d * (1 - cos(yaw*180/pi)))) - k_yaw*180/pi * yaw*180/pi * self.leader_w
        # angular_vel = self.leader_w + self.leader_x*(k_y*k_a*(ty+sin(yaw*180/pi)+k_yaw*180/pi*(yaw*180/pi)))+ k_b/k_a * sin(yaw*180/pi)



def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Formation Controller')
    parser.add_argument('--follower_type', choices=['mecanum', 'diffdrive', 'aruco'], required=True,
                        help='Specify follower robot type (mecanum or diffdrive)')
    args = parser.parse_args()

    formation_controller = FormationController(args.follower_type)
    rclpy.spin(formation_controller)
    formation_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
