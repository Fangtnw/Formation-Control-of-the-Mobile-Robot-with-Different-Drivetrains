import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_vector3
from math import cos , sin, pi


max_vx = 0.3
max_rz= 1.0

class FormationController(Node):
    def __init__(self):
        super().__init__('formation_controller')
        self.robot1_frame = 'base_footprint'
        self.robot2_frame = 'base_footprint_mec'
        self.leader_x = 0.0
        self.leader_rz = 0.0

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

        self.timer = self.create_timer(0.1, self.control_callback)

    def cmd_vel_callback(self, msg):
        self.leader_x = msg.linear.x 
        self.leader_rz = msg.angular.z 
        # pass

    def control_callback(self):
        try:
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
            linear_vel_x = (tx-1.5) * 5
            linear_vel_y = ty * 5
            linear_vel_x = max(min(linear_vel_x, max_vx), -max_vx)
            linear_vel_y = max(min(linear_vel_y, max_vx), -max_vx)
            
            if abs(tx-1.5) < 0.2 and abs(ty) < 0.2:
                # Adjust rz with a threshold and damping to avoid overcorrections:
                angular_vel = (rz - 1) * 50
            else:
                # If tx and ty are large, prioritize them, set angular_vel to 0
                angular_vel = 0.0
            angular_vel = max(min(angular_vel, max_rz), -max_rz)
            cmd_msg = Twist()
            cmd_msg.linear.x = linear_vel_x
            cmd_msg.linear.y = linear_vel_y
            cmd_msg.angular.z = angular_vel

            # Publish Twist message
            # self.publisher.publish(cmd_msg)
        except Exception as e:
            self.get_logger().error(f"Error looking up transform: {str(e)}")
            return

        d = 0
        k_a = 0.7
        k_b = 0.3
        k_x = 0.4
        k_y = 0.2    
        k_rz = 0.2

        # linear_vel = self.leader_x * cos(rz*180/pi) + (k_x * ((tx-1) - d * (1 - cos(rz*180/pi)))) - k_rz*180/pi * rz*180/pi * self.leader_rz
        # angular_vel = self.leader_rz + self.leader_x*(k_y*k_a*(ty+sin(rz*180/pi)+k_rz*180/pi*(rz*180/pi)))+ k_b/k_a * sin(rz*180/pi)



        # Limit linear velocity (respect robot's max speed)


def main(args=None):
    rclpy.init(args=args)
    formation_controller = FormationController()
    rclpy.spin(formation_controller)
    formation_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
