import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
import math
import numpy as np
import tf_transformations


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.subscription_plan = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10)
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/diffdrive/odom',
            self.odom_callback,
            10)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_speed = 0.25 # Set your desired speed here
        self.lookahead_distance = 1.1 # Set the lookahead distance here
        self.kp = 1  # Proportional control gain
        self.wheel_base = 0.5

        self.current_pose = None
        self.waypoints = None

        # Adding logger for debugging
        self.get_logger().info("Pure Pursuit Controller initialized")

    def plan_callback(self, msg):
        # Extract waypoints from the received Path message
        self.waypoints = msg.poses
        self.get_logger().info("Received new plan with {} waypoints".format(len(self.waypoints)))

    def odom_callback(self, msg):
        # Extract current pose from the received Odometry message
        self.current_pose = msg.pose.pose
        self.get_logger().info("Received new odometry data")

    def control(self):
        if self.current_pose is None or self.waypoints is None:
            return

        closest_index, closest_distance = self.find_closest_waypoint()

        # Calculate the lookahead index
        lookahead_index = closest_index
        if len(self.waypoints) > 0:
            total_distance = 0
            while total_distance < self.lookahead_distance and lookahead_index < len(self.waypoints) - 1:
                lookahead_index += 1
                total_distance += self.distance_between_points(
                    self.waypoints[lookahead_index].pose.position,
                    self.waypoints[lookahead_index - 1].pose.position
                )

        target_x = self.waypoints[lookahead_index].pose.position.x
        target_y = self.waypoints[lookahead_index].pose.position.y
        target_orientation = self.waypoints[lookahead_index].pose.orientation

        # Calculate errors
        delta_x = target_x - self.current_pose.position.x
        delta_y = target_y - self.current_pose.position.y
        distance_error = math.sqrt(delta_x ** 2 + delta_y ** 2)

        desired_heading = self.quaternion_to_yaw(target_orientation)

        current_orientation = self.quaternion_to_yaw(self.current_pose.orientation)

        alpha = desired_heading - current_orientation

        if alpha > math.pi:
            alpha -= 2 * math.pi
        elif alpha < -math.pi:
            alpha += 2 * math.pi
            
        steering_angle = math.atan2(2 * self.wheel_base * np.sin(abs(alpha)), total_distance)

        heading_sign = np.sign(np.sin(alpha))

        max_steering_angle = math.radians(35)
        steering_angle = max(min(steering_angle, max_steering_angle), -max_steering_angle) * heading_sign

        # Check if the robot is close to the last waypoint
        if lookahead_index == len(self.waypoints) - 1 and distance_error < self.lookahead_distance:
            linear_velocity = 0.0
            angular_velocity = 0.0
            steering_angle = 0.0  # Adjust steering angle for a smooth stop
        else:
            if abs(alpha) > 1.57:
                linear_velocity = min(self.target_speed, self.kp * distance_error)
                angular_velocity = linear_velocity / self.wheel_base * math.tan(steering_angle)
            else:
                linear_velocity = -min(self.target_speed, self.kp * distance_error)
                angular_velocity = -linear_velocity / self.wheel_base * math.tan(steering_angle)

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = angular_velocity
        self.publisher_cmd_vel.publish(cmd_vel_msg)

        self.get_logger().info("Control commands published: Linear Velocity = {}, Steering Angle = {}, Distance = {}, Alpha = {}".format(linear_velocity, steering_angle,total_distance,alpha))


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
    
def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_controller = PurePursuitController()
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
