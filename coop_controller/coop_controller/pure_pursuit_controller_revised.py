import rclpy
import rclpy.duration
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
import math
import numpy as np
import tf_transformations
import sys
import argparse
import tf2_ros
import tf2_geometry_msgs 
from rclpy.duration import Duration

class PurePursuitController(Node):
    def __init__(self, lookahead_distance, mode):
        super().__init__('pure_pursuit_controller')
        self.subscription_plan = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10)
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/ack/odom',
            self.odom_callback,
            10)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(seconds=5))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.target_speed = 0.2 # Set your desired speed here
        self.lookahead_distance = lookahead_distance
        self.mode = mode
        self.kp = 1  # Proportional control gain
        self.wheel_base = 0.5
        self.current_pose = None
        self.waypoints = None
        self.max_steering_angle = math.radians(20)
        self.reverse_mode = False
        self.get_logger().info("Pure Pursuit Controller initialized")

    def plan_callback(self, msg):
        # Extract waypoints from the received Path message
        self.waypoints = msg.poses
        for waypoint in self.waypoints:
            self.get_logger().info("Waypoint type: {}".format(type(waypoint)))
        self.get_logger().info("Received new plan with {} waypoints".format(len(self.waypoints)))

    def odom_callback(self, msg):
        # Extract current pose from the received Odometry message
        self.current_pose_odom = msg.pose.pose
        try:
            # Get the transform from the odom frame to the base_footprint frame
            transform = self.tf_buffer.lookup_transform('base_footprint', 'odom', rclpy.time.Time(), Duration(seconds=0.8))
            # Transform the current pose from odom frame to base_footprint frame
            self.current_pose = tf2_geometry_msgs.do_transform_pose(self.current_pose_odom, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error('Failed to get transform: {}'.format(e))
            return

    def get_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time(), Duration(seconds=0.8))
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error('Failed to get transform: {}'.format(e))
            return None

    def transform_waypoints(self, waypoints, transform):
        transformed_waypoints = []
        for waypoint in waypoints:
            self.get_logger().info("Waypoint type: {}".format(type(waypoint)))
            transformed_pose = tf2_geometry_msgs.do_transform_pose(waypoint, transform).pose
            transformed_waypoints.append(transformed_pose)
        return transformed_waypoints

        
    def control(self):
        if not self.current_pose or not self.waypoints:
            return

        # Get the transform from map to base_footprint
        transform = self.get_transform('base_footprint', 'map')
        if not transform:
            return

        # Transform waypoints to base_footprint frame
        transformed_waypoints = self.transform_waypoints(self.waypoints, transform)

        # Find the lookahead index using transformed waypoints
        lookahead_index = self.find_lookahead_index(transformed_waypoints)

        if lookahead_index < 0 or lookahead_index >= len(transformed_waypoints):
            lookahead_index = len(transformed_waypoints) - 1

        target_position = transformed_waypoints[lookahead_index].pose.position
        target_orientation = transformed_waypoints[lookahead_index].pose.orientation

        # Calculate error values in the robot frame
        delta_x = target_position.x
        delta_y = target_position.y
        self.distance_error = math.sqrt(delta_x ** 2 + delta_y ** 2)

        alpha = math.atan2(delta_y, delta_x)

        # Steering calculation
        steering_angle = math.atan2(2 * self.wheel_base * np.sin(alpha), self.distance_error)
        steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)

        # Determine if the robot should reverse
        if self.should_reverse(alpha, lookahead_index, transformed_waypoints):
            self.reverse_mode = not self.reverse_mode

        # Determine the desired linear and angular velocities
        if lookahead_index == len(transformed_waypoints) - 1 and self.distance_error < 0.5:
            linear_velocity = 0.0
            angular_velocity = 0.0
            steering_angle = 0.0
            self.get_logger().warn("stop at goal")
        else:
            if self.reverse_mode:
                linear_velocity = self.target_speed
            else:
                linear_velocity = -self.target_speed
            angular_velocity = linear_velocity / self.wheel_base * math.tan(steering_angle)

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = angular_velocity
        self.publisher_cmd_vel.publish(cmd_vel_msg)

        self.get_logger().info("Linear Velocity = {}, Steering Angle = {}, Distance = {}, Lookahead Index = {}".format(
            linear_velocity, steering_angle, self.distance_error, lookahead_index))

    def find_closest_waypoint(self):
        closest_index = 0
        closest_distance = float('inf')

        for i, waypoint in enumerate(self.waypoints):
            distance = self.distance_between_points(self.current_pose.position, waypoint.pose.position)
            if distance < closest_distance:
                closest_distance = distance
                closest_index = i

        return closest_index, closest_distance

    def distance_between_points(self, point1, point2):
        return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)

    def quaternion_to_yaw(self, quaternion):
        # Convert quaternion to yaw angle
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(q)
        return yaw

    def find_lookahead_index(self, waypoints):
        closest_index, _ = self.find_closest_waypoint()
        total_distance = 0.0

        for i in range(closest_index, len(waypoints)):
            distance = self.distance_between_points(self.current_pose.position, waypoints[i].pose.position)
            total_distance += distance
            if total_distance >= self.lookahead_distance:
                return i
        return len(waypoints) - 1

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def find_closest_cusp(self, waypoints):
        if not waypoints or len(waypoints) < 3:
            return -1
        closest_cusp_index = -1
        closest_distance = float('inf')

        for i in range(1, len(waypoints) - 1):
            oa_x = waypoints[i].pose.position.x - waypoints[i - 1].pose.position.x
            oa_y = waypoints[i].pose.position.y - waypoints[i - 1].pose.position.y
            ab_x = waypoints[i + 1].pose.position.x - waypoints[i].pose.position.x
            ab_y = waypoints[i + 1].pose.position.y - waypoints[i].pose.position.y

            dot_product = (oa_x * ab_x) + (oa_y * ab_y)

            if dot_product < 0:
                cusp_position = waypoints[i].pose.position
                distance_to_cusp = self.distance_between_points(self.current_pose.position, cusp_position)

                if distance_to_cusp < closest_distance:
                    closest_distance = distance_to_cusp
                    closest_cusp_index = i
        return closest_cusp_index

    def should_reverse(self, alpha, lookahead_index, waypoints):
        closest_cusp_index = self.find_closest_cusp(waypoints)
        if closest_cusp_index == -1:
            return False

        if lookahead_index >= closest_cusp_index and abs(alpha) > math.radians(90):
            self.get_logger().warn("Switching direction at cusp")
            return True

        return False

def main(args=None):
    parser = argparse.ArgumentParser(description='Pure Pursuit Controller')
    parser.add_argument('--lookahead', type=float, default=0.5, help='Set the lookahead distance')
    parser.add_argument('--mode', choices=['forward', 'reverse'], default='forward', help='Select mode: forward or reverse')
    args = parser.parse_args()
    
    rclpy.init()
    pure_pursuit_controller = PurePursuitController(lookahead_distance=args.lookahead, mode=args.mode)
    try:
        while rclpy.ok():
            rclpy.spin_once(pure_pursuit_controller)
            pure_pursuit_controller.control()
    except KeyboardInterrupt:
        pass

    pure_pursuit_controller.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
