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
            '/ack/odom',
            self.odom_callback,
            10)
    
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = tf2_ros.Buffer(Duration(seconds=0.1))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.target_speed = 0.06 # Set your desired speed here
        self.visited_cusps = set()
        self.lookahead_distance = lookahead_distance
        self.mode = mode
        if self.lookahead_distance is None:
            self.lookahead_distance = 0.5 # Set the lookahead distance here  0.8 for back 0.5 for front
        self.kp = 1  # Proportional control gain
        self.wheel_base = 0.5
        # self.lookahead_to_robot = 0.0
        self.prev_lookahead_to_robot = 0.0
        self.current_pose = None
        self.waypoints = None
        self.direction = 1
        self.prev_lookahead_to_robot = 0
        self.lookahead_index_set = False
        self.reverse_mode = False
        self.distance_error = 0
        self.cusp_stop = 0
        self.cusp_pass = 0
        self.cusp_found = 0
        self.max_steering_angle = math.radians(20)
        # Adding logger for debugging
        self.get_logger().info("Pure Pursuit Controller initialized")

    def plan_callback(self, msg):
        # Extract waypoints from the received Path message   
        self.waypoints = msg.poses
        self.get_logger().info("Received new plan with {} waypoints".format(len(self.waypoints)))
        
        
    def odom_callback(self, msg):
        # Extract current pose from the received Odometry message
        self.current_pose_odom = msg.pose.pose
        try:
            # Get the transform from the odom frame to the map frame
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time(), Duration(seconds=0.01))
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
        #     self.lookahead_index_set = True  # Mark the initial index as set
        # self.waypoints = transformed_waypoints
        og_lookahead_index = self.find_lookahead_index()
        closest_cusp_index = self.find_closest_cusp()

        if closest_cusp_index >= 0 and closest_cusp_index < og_lookahead_index :
            lookahead_index = closest_cusp_index
            self.get_logger().warn("found cusp let's get through it")
            # self.cusp_found = 1
        # elif self.cusp_stop == True:
        #     # loookahead after the cusp index
        #     # self.lookahead_distance = 1.5
        #     lookahead_index = self.find_next_lookahead_index(closest_cusp_index)
        #     self.get_logger().info("blind looking")
        #     if self.distance_error < 0.2:
        #         self.cusp_stop = 0
        #         self.cusp_pass = 1
        else:
            lookahead_index = og_lookahead_index
            self.get_logger().info("lookingahead with a normal mindset")
            # self.cusp_stop = False
            # self.direction = False


        # Ensure the lookahead index is within bounds
        if lookahead_index < 0 or lookahead_index >= len(self.waypoints):
            lookahead_index = len(self.waypoints)-1
            self.get_logger().info("Endpoint")
        
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
        
        steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)
        
        # if ((self.distance_error == 0 and lookahead_index < len(self.waypoints) - 1) or (alpha > 1.57) )and not self.reverse_mode:
        #     self.direction = self.direction * (-1) 
        #     self.reverse_mode = True
        # elif ((self.distance_error == 0 and lookahead_index < len(self.waypoints) - 1) or (alpha > 1.57) )and self.reverse_mode:
        #     self.direction = self.direction * (-1) 
        #     self.reverse_mode = False            

        self.direction = 1 if not self.reverse_mode else -1

        # Determine the desired linear and angular velocities
        if lookahead_index == len(self.waypoints) - 1 and (delta_y < 0.1 or self.distance_error < 0.1):
            linear_velocity = 0.0
            angular_velocity = 0.0
             
            self.get_logger().warn("stop at goal")
        elif lookahead_index == closest_cusp_index and delta_x < 0.5 and self.cusp_stop == 0:
            linear_velocity = 0.0
            angular_velocity = 0.0
             # Adjust steering angle for a smooth stop
            self.get_logger().warn("stop at cusp")
            self.reverse_mode = not self.reverse_mode 
            self.cusp_stop = 1
        # elif self.direction == -1 or self.reverse_mode is True:
            # self.lookahead_distance = 0.3
        #     linear_velocity = -self.target_speed
        #     angular_velocity = -linear_velocity / self.wheel_base * math.tan(steering_angle)
        elif self.mode == 'forward':
            linear_velocity = self.target_speed * self.direction
            angular_velocity = linear_velocity / self.wheel_base * math.tan(steering_angle) * self.direction
        elif self.mode == 'reverse':
            linear_velocity = -self.target_speed * self.direction
            angular_velocity = linear_velocity / self.wheel_base * math.tan(steering_angle) * self.direction

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = angular_velocity
        self.publisher_cmd_vel.publish(cmd_vel_msg)

        self.get_logger().info("Error = {:.2f},cusp = {:.2f},look= {}".format(self.distance_error,closest_cusp_index,lookahead_index))


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
    
    # def find_lookahead_index(self):
    #     total_distance = 0
    #     # Correctly unpack the tuple returned by find_closest_waypoint()
    #     closest_index, _ = self.find_closest_waypoint()
    #     lookahead_index = closest_index
        
    #     for i in range(lookahead_index, len(self.waypoints) - 1):
    #         total_distance += self.distance_between_points(
    #             self.waypoints[i].pose.position,
    #             self.waypoints[i + 1].pose.position
    #         )
            
    #         # Check if the total distance has reached or exceeded the lookahead distance
    #         if total_distance >= self.lookahead_distance:
    #             return lookahead_index
            
    #         # Increment the lookahead index after processing the current waypoint
    #         lookahead_index += 1
        
    #     return lookahead_index

    def find_lookahead_index(self):
        closest_index, _ = self.find_closest_waypoint()  # Find the closest waypoint
        for i in range(closest_index, len(self.waypoints)):
            distance = self.distance_between_points(self.current_pose.position, self.waypoints[i].pose.position)
            if distance >= self.lookahead_distance:
                return i  # Return the index of the waypoint that is at or beyond the lookahead distance
        return len(self.waypoints) - 1  # If no waypoint is beyond the lookahead distance, return the last waypoint


    def find_next_lookahead_index(self,current_index):
        total_distance = 0
        # Correctly unpack the tuple returned by find_closest_waypoint()
        lookahead_index = current_index
        
        for i in range(lookahead_index, len(self.waypoints) - 1):
            total_distance += self.distance_between_points(
                self.waypoints[i].pose.position,
                self.waypoints[i + 1].pose.position
            )
            
            # Check if the total distance has reached or exceeded the lookahead distance
            if total_distance >= self.lookahead_distance:
                return lookahead_index
            
            # Increment the lookahead index after processing the current waypoint
            lookahead_index += 1
        
        return lookahead_index
    
    def find_closest_cusp(self):
        if not self.waypoints or len(self.waypoints) < 3:
            return -1
        closest_cusp_index = -1
        closest_distance = float('inf')

        for i in range(1, len(self.waypoints) - 1):
            if i in self.visited_cusps: 
                continue
            oa_x = self.waypoints[i].pose.position.x - self.waypoints[i - 1].pose.position.x
            oa_y = self.waypoints[i].pose.position.y - self.waypoints[i - 1].pose.position.y
            ab_x = self.waypoints[i + 1].pose.position.x - self.waypoints[i].pose.position.x
            ab_y = self.waypoints[i + 1].pose.position.y - self.waypoints[i].pose.position.y

            dot_product = (oa_x * ab_x) + (oa_y * ab_y)

            if dot_product < 0:
                cusp_position = self.waypoints[i].pose.position
                distance_to_cusp = self.distance_between_points(self.current_pose.position, cusp_position)

                if distance_to_cusp < closest_distance:
                    closest_distance = distance_to_cusp
                    closest_cusp_index = i
        if closest_cusp_index != -1 and self.cusp_stop == 1:
            self.visited_cusps.add(closest_cusp_index)
            self.get_logger().warn("pass this cusp {}".format(closest_cusp_index))
            self.cusp_stop = 0
        return closest_cusp_index
    
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
            pure_pursuit_controller.control()
    except KeyboardInterrupt:
        pass

    pure_pursuit_controller.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
