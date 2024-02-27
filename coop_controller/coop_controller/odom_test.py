import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
import sys

class OdomTest(Node):
    def __init__(self, forward_distance_cm):
        super().__init__('odometry_test_node')
        self.get_logger().info('Odometry Test Node is running')
        self.initial_x = None
        self.current_x = 0.0
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_follower', 10)
        self.odom_sub = self.create_subscription(Odometry, '/mec_odom_raw', self.odom_callback, 10)
        self.FORWARD_DISTANCE_CM = forward_distance_cm

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x

        if self.initial_x is None:
            self.initial_x = self.current_x

        traveled_distance = abs(self.current_x - self.initial_x) * 100
        remaining_distance = abs(self.FORWARD_DISTANCE_CM) - traveled_distance

        if remaining_distance < 0:
            remaining_distance = 0

        self.get_logger().info(f'Remaining distance: {remaining_distance:.2f} cm')

        if traveled_distance >= abs(self.FORWARD_DISTANCE_CM):
            self.stop_robot()

    def move_robot(self, linear_x):
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        self.cmd_vel_pub.publish(twist_msg)

    def stop_robot(self):
        self.move_robot(0.0)

    def main(self):
        # Determine the direction based on the sign of FORWARD_DISTANCE_CM
        direction = 1 if self.FORWARD_DISTANCE_CM >= 0 else -1

        # Start moving the robot at the desired speed and direction
        self.move_robot(0.1 * direction)  # Adjust the speed as needed

        while rclpy.ok():
            rclpy.spin_once(self)

        # Shutdown ROS node after the loop exits
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    # Default value for FORWARD_DISTANCE_CM
    forward_distance_cm = 10

    # Check if a command-line argument is provided for FORWARD_DISTANCE_CM
    if len(sys.argv) > 1:
        try:
            forward_distance_cm = float(sys.argv[1])
        except ValueError:
            print("Invalid argument for FORWARD_DISTANCE_CM. Using default value.")

    odometry_test = OdomTest(forward_distance_cm)
    odometry_test.main()

if __name__ == '__main__':
    main()
