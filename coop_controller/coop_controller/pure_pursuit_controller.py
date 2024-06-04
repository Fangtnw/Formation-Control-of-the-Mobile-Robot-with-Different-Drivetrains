import rclpy
import rclpy.duration
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
import math
import numpy as np
import tf_transformations
import sys
import argparse
import tf2_ros
import geometry_msgs.msg
from rclpy.duration import Duration
import tf2_geometry_msgs

class PurePursuitController(Node):
    def __init__(self,lookahead_distance, mode):
        super().__init__('pure_pursuit_controller')
        self.subscription_plan = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10)
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/ack/odom',    #odometry/filtered /ack/odom
            self.odom_callback,
            10)
    
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel_pure', 1)
        self.publisher_lookahead_point = self.create_publisher(PointStamped, '/lookahead_point', 1)  # Add this line
        self.timer = self.create_timer(0.1, self.control)
        self.tf_buffer = tf2_ros.Buffer(Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.target_speed = 0.15 # Set your desired speed here
        self.visited_cusps = set()
        self.lookahead_distance = lookahead_distance
        self.mode = mode
        if self.lookahead_distance is None:
            self.lookahead_distance = 0.5 # Set the lookahead distance here  0.8 for back 0.5 for front
        self.kp = 1  # Proportional control gain
        self.wheel_base = 0.5
        # self.lookahead_to_robot = 0.0
        self.lookahead_index = None
        self.current_pose = None
        self.waypoints = None
        self.direction = 1
        self.prev_lookahead_to_robot = 0
        self.distance_error = 0
        self.cusp_stop = 0
        self.cusp_indices = []
        self.prev_closest_index = 0
        self.max_steering_angle = math.radians(40)
        self.cusp_reached = False
        self.cusp_timer = None
        self.prev_closest_index 
        
        # Adding logger for debugging
        self.get_logger().info("Pure Pursuit Controller initialized")

    def plan_callback(self, msg):
        # Extract waypoints from the received Path message   
        self.waypoints = msg.poses
        self.get_logger().info("Received new plan with {} waypoints".format(len(self.waypoints)))
        self.cusp_indices = self.find_closest_cusp()
        self.get_logger().info("Cusp indices: {}".format(self.cusp_indices))
        
    def odom_callback(self, msg):
        # Extract current pose from the received Odometry message
        self.current_pose_odom = msg.pose.pose
        try:
            # Get the transform from the odom frame to the map frame
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            # Transform the current pose from odom frame to map frame
            self.current_pose = tf2_geometry_msgs.do_transform_pose(self.current_pose_odom, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error('Failed to get transform: {}'.format(e))
            return
        # pass

    def control(self):
   
        if not self.current_pose or not self.waypoints:
            return

        # Transform the waypoints to the base_footprint frame
        # If lookahead index has not been set, find the closest waypoint
        # if not self.lookahead_index_set:
        #     self.lookahead_index = self.find_closest_waypoint()[0]
        #     self.lookahead_index_set = True  # Mark the initial index as setkkk
        # self.waypoints = transformed_waypoints
        og_lookahead_index = self.find_lookahead_index()
        closest_index, _ = self.find_closest_waypoint()
        closest_cusp_index = self.cusp_indices[0] if self.cusp_indices else -1

        if closest_cusp_index >= 0 and closest_cusp_index < og_lookahead_index :
            self.lookahead_index = closest_cusp_index
            self.get_logger().warn("found cusp")
        else:
            self.lookahead_index = og_lookahead_index
            self.get_logger().warn("normal look")
        
        if self.cusp_indices and (closest_index + 5 >= self.cusp_indices[0]) :
            self.cusp_reached = True
            self.cusp_indices.pop(0)
            self.get_logger().info("Cusp reached. Starting 3-second timer.")
            self.cusp_timer = self.create_timer(5.0, self.reset_cusp_reached)
        
        # if closest_index > closest_cusp_index and self.cusp_stop == 1:
        #     self.cusp_stop =0

        # Determine the position and orientation for the lookahead index
        target_position = self.waypoints[self.lookahead_index].pose.position
        target_orientation = self.waypoints[self.lookahead_index].pose.orientation

        lookahead_point_msg = PointStamped()
        lookahead_point_msg.header.frame_id = "map"
        lookahead_point_msg.header.stamp = self.get_clock().now().to_msg()
        lookahead_point_msg.point = target_position
        self.publisher_lookahead_point.publish(lookahead_point_msg) 

        # Calculate error values
        delta_x = target_position.x - self.current_pose.position.x
        delta_y = target_position.y - self.current_pose.position.y
        
        self.distance_error = math.sqrt(delta_x ** 2 + delta_y ** 2)

        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        desired_yaw = self.quaternion_to_yaw(target_orientation)

        # alpha = desired_yaw - current_yaw
        path_yaw = math.atan2(delta_y, delta_x)
        alpha = self.normalize_angle(path_yaw - current_yaw)

        target_point = PointStamped()
        target_point.header.frame_id = "map"
        target_point.point = target_position

        try:
            transform = self.tf_buffer.lookup_transform("base_footprint", "map", rclpy.time.Time())
            target_point_base = tf2_geometry_msgs.do_transform_point(target_point, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error('Failed to get transform: {}'.format(e))
            return

        # Use the x-coordinate of the target point in the base_footprint frame to set the direction
        self.direction = 1.0 if target_point_base.point.x >= 0.0 else -1.0

        # Steering calculation
        steering_angle = math.atan2(2 * self.wheel_base * math.sin(alpha), self.distance_error)
        steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle) 

        if self.lookahead_index == len(self.waypoints) - 1 and (self.distance_error < 0.15):
            linear_velocity = 0.0
            angular_velocity = 0.0
            self.get_logger().warn("stop at goal")
        elif self.cusp_reached:
            linear_velocity = self.target_speed * self.direction *-1.0
            angular_velocity = 0.0
            self.get_logger().warn("Passing through cusp ")
        else:
            linear_velocity = self.target_speed * self.direction
            angular_velocity = linear_velocity * math.tan(steering_angle) / self.wheel_base
        
        if self.lookahead_index < 0 or self.lookahead_index >= len(self.waypoints):
            self.lookahead_index = len(self.waypoints) - 1
            self.get_logger().info("Endpoint")
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = angular_velocity
        self.publisher_cmd_vel.publish(cmd_vel_msg)

        self.get_logger().info("Error = {:.2f},cusp = {:.2f},look= {}".format(closest_index,closest_cusp_index,self.lookahead_index))

    def reset_cusp_reached(self):
        self.cusp_reached = False
        self.cusp_timer.cancel()
        
        self.get_logger().info("3-second timer completed. Resuming normal operation.")
        self.cusp_timer = None

    # def find_closest_waypoint(self):
    #     closest_index = 0
    #     closest_distance = float('inf')

    #     for i, waypoint in enumerate(self.waypoints):
    #         distance = self.distance_between_points(self.current_pose.position, waypoint.pose.position)
    #         if distance < closest_distance:  
    #             closest_distance = distance
    #             closest_index = i

    #     return closest_index, closest_distance
    
    # def find_closest_waypoint(self):
    #     closest_distance = float('inf')
    #     closest_index = self.prev_closest_index 

    #     for i in range(self.prev_closest_index, len(self.waypoints)):
    #         distance = self.distance_between_points(self.current_pose.position, self.waypoints[i].pose.position)
    #         if distance < closest_distance:
    #             closest_distance = distance
    #             closest_index = i

    #     self.prev_closest_index = closest_index  # Update the previous closest index
    #     return closest_index, closest_distance
    
    def find_closest_waypoint(self):
        closest_distance = float('inf')
        closest_index = self.prev_closest_index
        max_index_jump = 25  # Define the maximum allowable jump in indices

        for i in range(self.prev_closest_index, len(self.waypoints)):
            distance = self.distance_between_points(self.current_pose.position, self.waypoints[i].pose.position)
            if distance < closest_distance:
                closest_distance = distance
                closest_index = i

        # Apply the max index jump threshold
        if abs(closest_index - self.prev_closest_index) > max_index_jump:
            if closest_index > self.prev_closest_index:
                closest_index = self.prev_closest_index 
            else:
                closest_index = i

        self.prev_closest_index = closest_index  # Update the previous closest index
        return closest_index, closest_distance

    
    def distance_between_points(self, point1, point2):
        return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)

    def quaternion_to_yaw(self, quaternion):
        # Convert quaternion to yaw angle
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(q)
        return yaw
    

    def find_lookahead_index(self):
        closest_index, _ = self.find_closest_waypoint()  # Find the closest waypoint
        if self.lookahead_index is not None:
            current_index = self.lookahead_index
        else:
            current_index = closest_index
        
        for i in range(current_index, len(self.waypoints)):
            distance = self.distance_between_points(self.current_pose.position, self.waypoints[i].pose.position)
            if distance >= self.lookahead_distance:
                return i  # Return the index of the waypoint that is at or beyond the lookahead distance
        return len(self.waypoints) - 1  # If no waypoint is beyond the lookahead distance, return the last waypoint

    
    def find_closest_cusp(self):
        if not self.waypoints or len(self.waypoints) < 3:
            return -1

        closest_index, _ = self.find_closest_waypoint()
        closest_cusp_index = -1
        cusp_indices = []

        # Find all cusp points
        for i in range(1, len(self.waypoints) - 1):
            oa_x = self.waypoints[i].pose.position.x - self.waypoints[i - 1].pose.position.x
            oa_y = self.waypoints[i].pose.position.y - self.waypoints[i - 1].pose.position.y
            ab_x = self.waypoints[i + 1].pose.position.x - self.waypoints[i].pose.position.x
            ab_y = self.waypoints[i + 1].pose.position.y - self.waypoints[i].pose.position.y

            dot_product = (oa_x * ab_x) + (oa_y * ab_y)

            if dot_product < 0:
                cusp_indices.append(i)

        # Find the closest cusp point that is beyond the closest waypoint index
        # for cusp_index in cusp_indices:
        #     if cusp_index > closest_index:
        #         closest_cusp_index = cusp_index
        #         break

        return cusp_indices

    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
def main(args=None):
    parser = argparse.ArgumentParser(description='Pure Pursuit Controller')

    # Add arguments for lookahead distance and mode
    parser.add_argument('--lookahead', type=float, default=0.5, help='Set the lookahead distance')
    parser.add_argument('--mode', choices=['forward', 'reverse'], default='forward', help='Select mode: forward or reverse')

    # Parse arguments
    args = parser.parse_args()
    
    rclpy.init()
    pure_pursuit_controller = PurePursuitController(lookahead_distance=args.lookahead, mode=args.mode)
    try:
        while rclpy.ok():
            rclpy.spin_once(pure_pursuit_controller)
            # pure_pursuit_controller.control()
    except KeyboardInterrupt:
        pass

    pure_pursuit_controller.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
