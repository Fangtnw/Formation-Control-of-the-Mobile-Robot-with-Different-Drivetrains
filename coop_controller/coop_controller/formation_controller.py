import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_vector3
from math import cos , sin, pi
import argparse

max_vx = 0.3
max_rz= 1.0

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
        self.leader_x = 0.0
        self.leader_rz = 0.0
        self.direction = 1.0
        self.previous_rz = 0
        self.previous_ty = 0
        self.follower_type = follower_type

        self.publisher = self.create_publisher(Twist, 'cmd_vel_follower', 10)
        #subscribe for leader Twist command
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.01, self.control_callback)

    def cmd_vel_callback(self, msg):
        self.leader_x = msg.linear.x 
        self.leader_rz = msg.angular.z 
        # pass

    def control_callback(self):
        try:
            cmd_msg = Twist()
            transform = self.tf_buffer.lookup_transform(
                self.robot1_frame, self.robot2_frame, rclpy.time.Time())
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z
            rx = transform.transform.rotation.x
            ry = transform.transform.rotation.y
            rz = transform.transform.rotation.z

            self.get_logger().info(
                f"Translation: x:{tx:.2f}, y:{ty:.2f}, z:{tz:.2f} , "
                f"Rotation: rx:{rx:.2f}, ry:{ry:.2f}, rz:{rz:.4f}" 
            )

            #mecanum_follower case
            if self.follower_type == 'mecanum':
                linear_vel_x = (tx - 1.5) * 5 
                linear_vel_y = ty * 5
                linear_vel_x = max(min(linear_vel_x, max_vx), -max_vx)
                linear_vel_y = max(min(linear_vel_y, max_vx), -max_vx)

                if abs(tx - 1.5) < 0.2 and abs(ty) < 0.2:
                    # Adjust rz with damping, considering previous direction:
                    if self.direction == 1.0 and abs(rz) < abs(self.previous_rz):   # Check turn direction
                        self.direction = -1.0 
                    elif self.direction == -1.0 and abs(rz) < abs(self.previous_rz):   # Check turn direction
                        self.direction = 1.0
                    angular_vel = (abs(rz) - 1) * 50  * self.direction  
                else:
                    angular_vel = 0.0  # Prioritize x, y if they are large
                cmd_msg.linear.y = linear_vel_y
                angular_vel = max(min(angular_vel, max_rz), -max_rz)

            #differential_drive robot follower case
            elif self.follower_type == 'diffdrive':
                # linear_vel_x =  ty * 5  
                # if self.direction == 1.0 and abs(ty) < abs(self.previous_ty):   # Check turn direction
                #     self.direction = -1.0 
                # elif self.direction == -1.0 and abs(ty) < abs(self.previous_ty):   # Check turn direction
                #     self.direction = 1.0
                # angular_vel = ty * 5  * self.direction
                    
                # if abs(ty) < 0.2:
                #     if self.direction == 1.0 and abs(rz) < abs(self.previous_rz):   # Check turn direction
                #         self.direction = -1.0 
                #     elif self.direction == -1.0 and abs(rz) < abs(self.previous_rz):   # Check turn direction
                #         self.direction = 1.0
                #     angular_vel = (abs(rz) - 1) * 50  * self.direction  
                
                linear_vel_x = (tx-1.5) * 5  
                if abs(tx - 1.5) < 0.2 :
                    if self.direction == 1.0 and abs(rz) < abs(self.previous_rz):   # Check turn direction
                        self.direction = -1.0 
                    elif self.direction == -1.0 and abs(rz) < abs(self.previous_rz):   # Check turn direction
                        self.direction = 1.0
                    angular_vel = (abs(rz) - 1) * 50  * self.direction  
                else:
                    angular_vel = 0.0

                linear_vel_x = max(min(linear_vel_x, max_vx), -max_vx)
                angular_vel = max(min(angular_vel, max_rz), -max_rz)
            else:
                self.get_logger().error(f"Invalid follower type '{self.follower_type}'")
                exit()

            cmd_msg.linear.x = linear_vel_x
            cmd_msg.angular.z = angular_vel

            self.publisher.publish(cmd_msg)
            self.previous_ty = ty 
            self.previous_rz = rz 

        except Exception as e:
            self.get_logger().error(f"Error looking up transform: {str(e)}")
            return

        # linear_vel = self.leader_x * cos(rz*180/pi) + (k_x * ((tx-1) - d * (1 - cos(rz*180/pi)))) - k_rz*180/pi * rz*180/pi * self.leader_rz
        # angular_vel = self.leader_rz + self.leader_x*(k_y*k_a*(ty+sin(rz*180/pi)+k_rz*180/pi*(rz*180/pi)))+ k_b/k_a * sin(rz*180/pi)


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Formation Controller')
    parser.add_argument('--follower_type', choices=['mecanum', 'diffdrive'], required=True,
                        help='Specify follower robot type (mecanum or diffdrive)')
    args = parser.parse_args()

    formation_controller = FormationController(args.follower_type)
    rclpy.spin(formation_controller)
    formation_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
