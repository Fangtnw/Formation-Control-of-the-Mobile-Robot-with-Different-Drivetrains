import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import numpy as np
import matplotlib.pyplot as plt
import signal
from scipy.spatial import distance
import tf_transformations
import tf2_ros
import geometry_msgs.msg
from rclpy.duration import Duration
import tf2_geometry_msgs
import csv
import os

def get_unique_filename(base_name, extension):
    """
    Generate a unique filename by appending a number if the base name already exists.
    """
    counter = 1
    file_name = f"{base_name}.{extension}"
    while os.path.exists(file_name):
        file_name = f"{base_name}_{counter}.{extension}"
        counter += 1
    return file_name

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_follower', self.cmd_vel_follower_callback, 10)
        self.create_subscription(Odometry, '/ack/odom', self.ack_odom_callback, 10)
        self.create_subscription(Twist, 'ack_encoder_vel', self.ack_encoder_vel_callback, 1)
        self.create_subscription(Twist, '/mec_encoder_vel', self.mec_encoder_vel_callback, 1)        
        self.create_subscription(Odometry, '/mec/odom', self.mec_odom_callback, 10)
        self.create_subscription(Path, '/plan', self.path_callback, 10)

        self.is_moving = False  # Indicates whether the robot is moving
        self.start_times = []  # To track multiple start times
        self.end_times = []  # To track multiple end times

        self.leader_x_positions = []
        self.leader_y_positions = []
        self.leader_yaws = []

        self.follower_x_positions = []
        self.follower_y_positions = []
        self.follower_yaws = []

        self.leader_follower_x_positions =[]
        self.leader_follower_y_positions = []
        self.leader_follower_yaws = []

        self.path_x_positions = []
        self.path_y_positions = []
        self.path_yaws = [] 

        self.leader_velocities = []
        self.follower_velocities = []

        self.leader_encoder = []
        self.follower_encoder = []


        self.leader_times = []
        self.follower_times = []
        
        self.tf_buffer = tf2_ros.Buffer(Duration(seconds=2))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.reference_time = self.get_clock().now().nanoseconds / 1e9

    def ack_odom_callback(self, msg):
        self.current_pose_ack = msg.pose.pose
        try:
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            transformed_msg = tf2_geometry_msgs.do_transform_pose(self.current_pose_ack, transform)
            self.leader_x_positions.append(transformed_msg.position.x)
            self.leader_y_positions.append(transformed_msg.position.y)
            self.leader_yaws.append(self.quaternion_to_yaw(transformed_msg.orientation))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn('Transform ACK lookup failed')
            # pass
        try:
            transform = self.tf_buffer.lookup_transform('map', 'camera_frame', rclpy.time.Time())
            # transformed_msg = tf2_geometry_msgs.do_transform_pose(self.current_pose_mec, transform)
            self.follower_x_positions.append(transform.transform.translation.x)
            self.follower_y_positions.append(transform.transform.translation.y)
            self.follower_yaws.append(self.quaternion_to_yaw(transform.transform.rotation))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn('Transform MEC lookup failed')

            # pass
    
    def mec_odom_callback(self, msg):
        # self.current_pose_mec = msg.pose.pose
        pass

    def path_callback(self, msg):
        self.path_x_positions = [pose.pose.position.x for pose in msg.poses]
        self.path_y_positions = [pose.pose.position.y for pose in msg.poses]
        self.path_yaws = [
            self.quaternion_to_yaw(pose.pose.orientation) for pose in msg.poses
        ]

    def cmd_vel_callback(self, msg):
        if self.reference_time is None:
            self.reference_time = self.get_clock().now().nanoseconds / 1e9
        
        try:
            transform = self.tf_buffer.lookup_transform('base_footprint', 'base_footprint_mec', rclpy.time.Time())
            # transformed_msg = tf2_geometry_msgs.do_transform_pose(self.current_pose_mec, transform)
            self.leader_follower_x_positions.append(transform.transform.translation.x)
            self.leader_follower_y_positions.append(transform.transform.translation.y)
            self.leader_follower_yaws.append(self.quaternion_to_yaw(transform.transform.rotation))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn('Transform MEC lookup failed')

        current_time = self.get_clock().now().nanoseconds / 1e9 - self.reference_time
        self.leader_velocities.append((current_time, msg.linear.x, msg.angular.z))
        self.leader_times.append(current_time)  # Recording the time for leader

        is_currently_moving = (msg.linear.x != 0.0 or msg.angular.z != 0.0)

        if not self.is_moving and is_currently_moving:
            self.is_moving = True
            start_time = current_time
            self.start_times.append(start_time)
            self.get_logger().info("Robot started moving at time: {:.2f} seconds".format(start_time))
        
        elif self.is_moving and not is_currently_moving:
            self.is_moving = False
            end_time = current_time
            self.end_times.append(end_time)
            self.get_logger().info("Robot stopped moving at time: {:.2f} seconds".format(end_time))


    def cmd_vel_follower_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9 - self.reference_time  # Adjust to use the same reference time
        self.follower_velocities.append((current_time, msg.linear.x, msg.linear.y, msg.angular.z))
        self.follower_times.append(current_time)  # Recording the time for follower


    def mec_encoder_vel_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9  # Time in seconds
        self.follower_motor1_vel = msg.linear.x
        self.follower_motor2_vel = msg.linear.y
        self.follower_motor3_vel = msg.angular.x
        self.follower_motor4_vel = msg.angular.y
        self.follower_encoder.append((current_time, msg.linear.x, msg.linear.y, msg.angular.x, msg.angular.y))

    def ack_encoder_vel_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9  # Time in seconds
        self.leader_left_vel = msg.linear.y
        self.leader_right_vel =  msg.linear.x
        self.leader_steering_angle = -msg.angular.y
        self.leader_encoder.append((current_time, msg.linear.y, msg.linear.x, -msg.angular.y))

    def calculate_errors(self, x_positions, y_positions, reference_x, reference_y, times):
        if not self.path_x_positions:
            return {'dist': [], 'mae': 0, 'rmse': 0, 'max_error': 0, 'min_error': 0}
        
        distances = [
            (time, min(distance.euclidean((x, y), (rx, ry)) for rx, ry in zip(reference_x, reference_y)))
            for time, (x, y) in zip(times, zip(x_positions, y_positions))
        ]
        error_values = [dist for time, dist in distances]
        
        return {
            'dist': distances,
            'mae': np.mean(error_values),
            'rmse': np.sqrt(np.mean(np.square(error_values))),
            'max_error': np.max(error_values),
            'min_error': np.min(error_values),
        }
    
    def calculate_cross_track_error(self, x_positions, y_positions, reference_x, reference_y):
        # Calculate the perpendicular distance to the path (cross-track error)
        cross_track_errors = [
            min(distance.euclidean((x, y), (rx, ry)) for rx, ry in zip(reference_x, reference_y))
            for x, y in zip(x_positions, y_positions)
        ]
        return cross_track_errors

    def calculate_total_operate_time(self):
        # Ensure there are matching pairs of start and end times
        if len(self.start_times) == len(self.end_times):
            total_operate_time = sum(
                end - start for start, end in zip(self.start_times, self.end_times)
            )
            return total_operate_time
        return None

    def calculate_last_point_error(self):
        if not self.path_x_positions or not self.leader_x_positions or not self.follower_x_positions:
            return None

        # Get the last position of the leader, follower, and reference path
        leader_last = (self.leader_x_positions[-1], self.leader_y_positions[-1], self.leader_yaws[-1])
        follower_last = (self.follower_x_positions[-1], self.follower_y_positions[-1], self.follower_yaws[-1])

        # Use the closest path point as the reference
        path_last_idx = np.argmin([
            distance.euclidean((x, y), (self.leader_x_positions[-1], self.leader_y_positions[-1]))
            for x, y in zip(self.path_x_positions, self.path_y_positions)
        ])

        path_last = (
            self.path_x_positions[path_last_idx], 
            self.path_y_positions[path_last_idx], 
            self.path_yaws[path_last_idx]
        )

        # Calculate errors in x, y, and yaw for the leader and follower
        def calc_positional_error(point1, point2):
            # Calculate the x, y, and yaw errors between two points
            x_error = point1[0] - point2[0]
            y_error = point1[1] - point2[1]
            yaw_error = point1[2] - point2[2]
            return {'x_error': x_error, 'y_error': y_error, 'yaw_error': yaw_error}

        leader_error = calc_positional_error(leader_last, path_last)
        follower_error = calc_positional_error(follower_last, path_last)

        return leader_error, follower_error

    def calculate_leader_follower_error(self):
        if not self.leader_x_positions or not self.follower_x_positions:
            return {'dist': [], 'mae': 0, 'rmse': 0, 'max_error': 0, 'min_error': 0}

        # Calculate the positional errors between the leader and follower
        positional_errors = [
            distance.euclidean((lx, ly), (fx, fy))
            for lx, ly, fx, fy in zip(self.leader_x_positions, self.leader_y_positions, self.follower_x_positions, self.follower_y_positions)
        ]

        return {
            'dist': positional_errors,
            'mae': np.mean(positional_errors),  # Mean Absolute Error
            'rmse': np.sqrt(np.mean(np.square(positional_errors))),  # Root Mean Square Error
            'max_error': np.max(positional_errors),  # Maximum Error
            'min_error': np.min(positional_errors),  # Minimum Error
        }

    def plot_trajectory(self):
        plt.figure(figsize=(8, 6))
        if self.path_x_positions and self.path_y_positions:
            plt.plot(
                self.path_x_positions, 
                self.path_y_positions, 
                label='Planned Path', 
                color='blue',
                linewidth=2,  
                linestyle='-'  
            )
        plt.plot(
            self.leader_x_positions, 
            self.leader_y_positions, 
            label='Leader Trajectory', 
            color='orange',
            linewidth=2,  
            linestyle='--'  
        )
        plt.plot(
            self.follower_x_positions, 
            self.follower_y_positions, 
            label='Follower Trajectory', 
            color='green',
            linewidth=2,  
            linestyle='--'  
        )

        # Calculate total operating time
        total_operate_time = self.calculate_total_operate_time()

        # Calculate additional operate time since the first movement
        if self.start_times:
            first_start_time = min(self.start_times)
            latest_end_time = max(self.end_times) if self.end_times else first_start_time
            additional_operate_time = latest_end_time - first_start_time
            print("Additional operate time since the first movement:", additional_operate_time, "seconds")

        if self.path_x_positions and self.path_y_positions :
            # Calculate error metrics if the path is available
            leader_errors = self.calculate_errors(
                self.leader_x_positions, 
                self.leader_y_positions, 
                self.path_x_positions, 
                self.path_y_positions,
                 self.leader_times
            )
            leader_cross_track_errors = self.calculate_cross_track_error(
                self.leader_x_positions, 
                self.leader_y_positions, 
                self.path_x_positions, 
                self.path_y_positions
            )
            print("Leader Errors:")
            print(f"  - MAE: {leader_errors['mae']:.2f}")
            print(f"  - RMSE: {leader_errors['rmse']:.2f}")
            print(f"  - Maximum Error: {leader_errors['max_error']:.2f}")
            print(f"  - Minimum Error: {leader_errors['min_error']:.2f}")
            print(f"  - Cross Track Error: {np.mean(leader_cross_track_errors):.2f}")
            if self.follower_x_positions:
                follower_errors = self.calculate_errors(
                    self.follower_x_positions, 
                    self.follower_y_positions, 
                    self.path_x_positions, 
                    self.path_y_positions,
                     self.follower_times
                )
                follower_cross_track_errors = self.calculate_cross_track_error(
                    self.follower_x_positions, 
                    self.follower_y_positions, 
                    self.path_x_positions, 
                    self.path_y_positions
                )
                follower_leader_errors = self.calculate_leader_follower_error()
                print("Follower Errors:")
                print(f"  - MAE: {follower_errors['mae']:.2f}")
                print(f"  - RMSE: {follower_errors['rmse']:.2f}")
                print(f"  - Maximum Error: {follower_errors['max_error']:.2f}")
                print(f"  - Minimum Error: {follower_errors['min_error']:.2f}")
                print(f"  - Cross Track Error: {np.mean(follower_cross_track_errors):.2f}")

                if follower_leader_errors:
                    print("Follower-Leader Errors:")
                    print(f"  - MAE: {follower_leader_errors['mae']:.2f}")
                    print(f"  - RMSE: {follower_leader_errors['rmse']:.2f}")
                    print(f"  - Maximum Error: {follower_leader_errors['max_error']:.2f}")
                    print(f"  - Minimum Error: {follower_leader_errors['min_error']:.2f}")

        # Always calculate leader-follower error

        if total_operate_time is not None:
            print("Total operate time:", total_operate_time, "seconds")

        last_point_errors = self.calculate_last_point_error()
        if last_point_errors:
            leader_error, follower_error = last_point_errors

            print("Last Point Errors:")
            print("  Leader:")
            print(f"    - X Error: {leader_error['x_error']:.2f}")
            print(f"    - Y Error: {leader_error['y_error']:.2f}")
            print(f"    - Yaw Error: {leader_error['yaw_error']:.2f}")

            print("  Follower:")
            print(f"    - X Error: {follower_error['x_error']:.2f}")
            print(f"    - Y Error: {follower_error['y_error']:.2f}")
            print(f"    - Yaw Error: {follower_error['yaw_error']:.2f}")

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Comparison of Planned and Actual Trajectories')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

    def save_velocities_to_csv(self):
        unique_filename = get_unique_filename('experiment', 'csv')
        with open(unique_filename, 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow([
                'Time (s)', 'Leader Linear X', 'Leader Angular Z', 'Leader Left Vel', 'Leader Right Vel', 'Leader Steering Angle', 
                'Follower Linear X', 'Follower Linear Y', 'Follower Angular Z', 'Follower Motor1 Vel', 'Follower Motor2 Vel', 'Follower Motor3 Vel', 'Follower Motor4 Vel',
                'Leader Dist', 'Leader MAE', 'Leader RMSE', 'Leader Max Error', 'Leader Min Error',
                'Follower Dist', 'Follower MAE', 'Follower RMSE', 'Follower Max Error', 'Follower Min Error',
                'Formation Dist', 'Formation MAE', 'Formation RMSE', 'Formation Max Error', 'Formation Min Error',
                'Total Operate Time', 'Leader-Follower X', 'Leader-Follower Y', 'Leader-Follower Yaw'
            ])

            leader_errors = self.calculate_errors(self.leader_x_positions, self.leader_y_positions, self.path_x_positions, self.path_y_positions, self.leader_times)
            follower_errors = {'dist': [], 'mae': 0, 'rmse': 0, 'max_error': 0, 'min_error': 0}
            follower_leader_errors = {'dist': [], 'mae': 0, 'rmse': 0, 'max_error': 0, 'min_error': 0}

            if self.follower_x_positions:
                follower_errors = self.calculate_errors(self.follower_x_positions, self.follower_y_positions, self.path_x_positions, self.path_y_positions, self.follower_times)
                follower_leader_errors = self.calculate_leader_follower_error()

            total_operate_time = self.calculate_total_operate_time()
            max_len = max(len(self.leader_velocities), len(self.follower_velocities), len(self.leader_encoder), len(self.follower_encoder), len(self.leader_follower_x_positions))

            for i in range(max_len):
                leader_time, leader_linear_x, leader_angular_z = self.leader_velocities[i] if i < len(self.leader_velocities) else ('', '', '')
                follower_time, follower_linear_x, follower_linear_y, follower_angular_z = self.follower_velocities[i] if i < len(self.follower_velocities) else ('', '', '', '')
                _, leader_left_vel, leader_right_vel, leader_steering_angle = self.leader_encoder[i] if i < len(self.leader_encoder) else ('', '', '', '')
                _, follower_motor1_vel, follower_motor2_vel, follower_motor3_vel, follower_motor4_vel = self.follower_encoder[i] if i < len(self.follower_encoder) else ('', '', '', '', '')
                leader_follower_x = self.leader_follower_x_positions[i] if i < len(self.leader_follower_x_positions) else ''
                leader_follower_y = self.leader_follower_y_positions[i] if i < len(self.leader_follower_y_positions) else ''
                leader_follower_yaw = self.leader_follower_yaws[i] if i < len(self.leader_follower_yaws) else ''

                leader_dist, follower_dist, follower_leader_dist = '', '', ''
                if i < len(leader_errors['dist']):
                    leader_dist = leader_errors['dist'][i][1]
                if i < len(follower_errors['dist']):
                    follower_dist = follower_errors['dist'][i][1]
                if i < len(follower_leader_errors['dist']):
                    follower_leader_dist = follower_leader_errors['dist'][i]

                csvwriter.writerow([
                    leader_time, leader_linear_x, leader_angular_z, leader_left_vel, leader_right_vel, leader_steering_angle,
                    follower_linear_x, follower_linear_y, follower_angular_z, follower_motor1_vel, follower_motor2_vel, follower_motor3_vel, follower_motor4_vel,
                    leader_dist, leader_errors['mae'] if i == 0 else '', leader_errors['rmse'] if i == 0 else '', leader_errors['max_error'] if i == 0 else '', leader_errors['min_error'] if i == 0 else '',
                    follower_dist, follower_errors['mae'] if i == 0 else '', follower_errors['rmse'] if i == 0 else '', follower_errors['max_error'] if i == 0 else '', follower_errors['min_error'] if i == 0 else '',
                    follower_leader_dist, follower_leader_errors['mae'] if i == 0 else '', follower_leader_errors['rmse'] if i == 0 else '', follower_leader_errors['max_error'] if i == 0 else '', follower_leader_errors['min_error'] if i == 0 else '',
                    total_operate_time if i == 0 else '', leader_follower_x, leader_follower_y, leader_follower_yaw
                ])

        self.get_logger().info(f"Saved velocities and errors to CSV file: {unique_filename}")




    def quaternion_to_yaw(self, quaternion):
        # Convert quaternion to yaw angle
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(q)
        return yaw
    
    def shutdown_callback(self):
        self.plot_trajectory()
        self.save_velocities_to_csv()
        self.get_logger().info("Saved velocities to CSV and generated plot.")
        # rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    trajectory_plotter = TrajectoryPlotter()
    signal.signal(signal.SIGINT, lambda sig, frame: trajectory_plotter.shutdown_callback())
    rclpy.spin(trajectory_plotter)
    trajectory_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
