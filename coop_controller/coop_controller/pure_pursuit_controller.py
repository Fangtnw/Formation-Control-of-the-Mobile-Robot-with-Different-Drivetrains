import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
import math
import numpy as np
import tf_transformations
import sys


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
        self.lookahead_distance = 0.5 # Set the lookahead distance here  1.1
        self.kp = 1  # Proportional control gain
        self.wheel_base = 0.5
        # self.lookahead_to_robot = 0.0
        self.prev_lookahead_to_robot = 0.0
        self.current_pose = None
        self.waypoints = None
        self.direction = 1
        self.prev_lookahead_to_robot = 0
        self.lookahead_index_set = False
        self.distance_error = 0
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
        if not self.current_pose or not self.waypoints:
            return

        # If lookahead index has not been set, find the closest waypoint
        if not self.lookahead_index_set:
            self.lookahead_index = self.find_closest_waypoint()[0]
            self.lookahead_index_set = True  # Mark the initial index as set
        
        og_lookahead_index = self.find_lookahead_index(self.lookahead_index,self.distance_error)
        closest_cusp_index = self.find_closest_cusp()

        if closest_cusp_index >= 0 and closest_cusp_index < og_lookahead_index:
            lookahead_index = closest_cusp_index
            self.get_logger().warn("found cusp")
            # self.direction = True
        else:
            lookahead_index = og_lookahead_index
            # self.direction = False

            
        # Ensure the lookahead index is within bounds
        if lookahead_index < 0 or lookahead_index >= len(self.waypoints):
            lookahead_index = len(self.waypoints)-1
        
        # Determine the position and orientation for the lookahead index
        target_position = self.waypoints[lookahead_index].pose.position
        target_orientation = self.waypoints[lookahead_index].pose.orientation
        
        # Calculate error values
        delta_x = target_position.x - self.current_pose.position.x
        delta_y = target_position.y - self.current_pose.position.y
        self.distance_error = math.sqrt(delta_x ** 2 + delta_y ** 2)

        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        desired_yaw = self.quaternion_to_yaw(target_orientation)

        alpha = desired_yaw - current_yaw
        alpha = self.normalize_angle(alpha)

        # Steering calculation
        steering_angle = math.atan2(2 * self.wheel_base * np.sin(alpha), self.distance_error)
        max_steering_angle = math.radians(35)
        steering_angle = max(min(steering_angle, max_steering_angle), -max_steering_angle)
        
        if self.distance_error == 0 and lookahead_index < len(self.waypoints) - 1:
            self.direction = self.direction * (-1)

        # Determine the desired linear and angular velocities
        if lookahead_index == len(self.waypoints) - 1 and self.distance_error < 0.1:
            linear_velocity = 0.0
            angular_velocity = 0.0
            steering_angle = 0.0  # Adjust steering angle for a smooth stop
        elif self.direction == -1:
            linear_velocity = self.target_speed
            angular_velocity = linear_velocity / self.wheel_base * math.tan(steering_angle)
        else:
            linear_velocity = -self.target_speed
            angular_velocity = -linear_velocity / self.wheel_base * math.tan(steering_angle)

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = angular_velocity
        self.publisher_cmd_vel.publish(cmd_vel_msg)

        self.get_logger().info("Control commands published: Linear Velocity = {}, Steering Angle = {}, Distance = {}, Alpha = {}".format(linear_velocity, steering_angle,self.distance_error,alpha))


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
    
    def find_lookahead_index(self,lookahead_index,total_distance):
        
        for i in range(len(self.waypoints) - 1):
            lookahead_index += 1
            total_distance += self.distance_between_points(
                self.waypoints[i].pose.position,
                self.waypoints[i + 1].pose.position
            )
            if total_distance >= self.lookahead_distance:
                return lookahead_index
        return lookahead_index  
    
    def find_closest_cusp(self):
        """Returns the index of the closest cusp to the robot, or -1 if no cusps are found."""
        if not self.waypoints or len(self.waypoints) < 3:
            # Not enough waypoints to find cusps
            return -1
        
        closest_cusp_index = -1
        closest_distance = float('inf')

        for i in range(1, len(self.waypoints) - 1):
            # Calculate vectors between consecutive waypoints
            oa_x = self.waypoints[i].pose.position.x - self.waypoints[i - 1].pose.position.x
            oa_y = self.waypoints[i].pose.position.y - self.waypoints[i - 1].pose.position.y
            ab_x = self.waypoints[i + 1].pose.position.x - self.waypoints[i].pose.position.x
            ab_y = self.waypoints[i + 1].pose.position.y - self.waypoints[i].pose.position.y

            # Determine if there's a cusp
            dot_product = (oa_x * ab_x) + (oa_y * ab_y)
            
            if dot_product < 0:
                # If dot product is negative, it indicates a cusp
                cusp_position = self.waypoints[i].pose.position
                distance_to_cusp = self.distance_between_points(self.current_pose.position, cusp_position)
                
                if distance_to_cusp < closest_distance:
                    closest_distance = distance_to_cusp
                    closest_cusp_index = i

        return closest_cusp_index
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
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
