import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import math
import tf_transformations

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.subscription = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/diffdrive/odom',
            self.odom_callback,
            10)
        self.odom_subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_data = None
        self.path = None
        self.next_waypoint_index = 0
        self.rotating = False

    def plan_callback(self, msg):
        self.get_logger().info('Received new path plan')
        self.path = msg
        self.next_waypoint_index = 0

    def odom_callback(self, msg):
        self.odom_data = msg
        self.follow_path()

    def follow_path(self):
        if self.odom_data is None or self.path is None:
            return

        current_x = self.odom_data.pose.pose.position.x
        current_y = self.odom_data.pose.pose.position.y
        current_theta = self.quaternion_to_yaw(self.odom_data.pose.pose.orientation)

        # Check if all waypoints have been reached
        if self.next_waypoint_index >= len(self.path.poses):
            self.get_logger().info('All waypoints reached. Path following complete.')
            return

        target_pose = self.path.poses[self.next_waypoint_index].pose
        target_x = target_pose.position.x
        target_y = target_pose.position.y
        target_theta = self.quaternion_to_yaw(target_pose.orientation)

        distance_to_target = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

        # If the robot is close to the target waypoint and orientation is correct
        if distance_to_target <= 0.1 and abs(target_theta - current_theta) < 0.1:
            self.next_waypoint_index += 1
            self.get_logger().info('Moving to next waypoint')
            return

        # If the robot is not close to the target waypoint, check orientation
        if abs(target_theta - current_theta) >= 0.01:
            self.rotate_to_target(target_theta, current_theta)
            self.get_logger().info('Rotating to align with the path')
            self.get_logger().info('Distance to rotate: %.2f' % abs(target_theta - current_theta))
        else:
            self.move_to_target(target_x, target_y)
            self.get_logger().info('Moving towards the next waypoint. Distance to target: %.2f' % distance_to_target)
            
    def rotate_to_target(self, target_theta, current_theta):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.5 if target_theta > current_theta else -0.5  # rotate clockwise or counter-clockwise based on the difference between target and current theta
        self.publisher_.publish(twist_msg)
        # Publish the target theta and current theta
        self.get_logger().info('Published Target Theta: %.2f, Current Theta: %.2f' % (target_theta, current_theta))

    def move_to_target(self, target_x, target_y):
        twist_msg = Twist()
        twist_msg.linear.x = 0.5  # move forward with a constant linear velocity
        self.publisher_.publish(twist_msg)

    def quaternion_to_yaw(self, quaternion):
        # Convert quaternion to yaw angle
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(q)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    path_follower = PathFollower()
    rclpy.spin(path_follower)
    path_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
