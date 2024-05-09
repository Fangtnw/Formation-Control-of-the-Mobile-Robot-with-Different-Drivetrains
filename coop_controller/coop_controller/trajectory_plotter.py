import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import numpy as np
import matplotlib.pyplot as plt
import signal
from scipy.spatial import distance
import tf_transformations

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/diffdrive/odom', self.diffdrive_odom_callback, 10)
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

        self.path_x_positions = []
        self.path_y_positions = []
        self.path_yaws = [] 

    def diffdrive_odom_callback(self, msg):
        self.leader_x_positions.append(msg.pose.pose.position.x)
        self.leader_y_positions.append(msg.pose.pose.position.y)
        self.leader_yaws.append(self.quaternion_to_yaw(msg.pose.pose.orientation))

    def mec_odom_callback(self, msg):
        self.follower_x_positions.append(msg.pose.pose.position.x)
        self.follower_y_positions.append(msg.pose.pose.position.y)
        self.follower_yaws.append(self.quaternion_to_yaw(msg.pose.pose.orientation))


    def path_callback(self, msg):
        self.path_x_positions = [pose.pose.position.x for pose in msg.poses]
        self.path_y_positions = [pose.pose.position.y for pose in msg.poses]
        self.path_yaws = [
            self.quaternion_to_yaw(pose.pose.orientation) for pose in msg.poses
        ]

    def cmd_vel_callback(self, msg):
        # Check if the robot is moving or has stopped
        is_currently_moving = (msg.linear.x != 0.0 or msg.angular.z != 0.0)

        # If robot wasn't moving but now is, record the start time
        if not self.is_moving and is_currently_moving:
            self.is_moving = True
            start_time = self.get_clock().now().nanoseconds / 1e9  # Time in seconds
            self.start_times.append(start_time)  # Append to start times list
            self.get_logger().info("Robot started moving at time: {:.2f} seconds".format(start_time))
        
        # If robot was moving but has stopped, record the end time
        elif self.is_moving and not is_currently_moving:
            self.is_moving = False
            end_time = self.get_clock().now().nanoseconds / 1e9
            self.end_times.append(end_time)  # Append to end times list
            self.get_logger().info("Robot stopped moving at time: {:.2f} seconds".format(end_time))

    def calculate_errors(self, x_positions, y_positions, reference_x, reference_y):
        # Calculate distances to reference points
        distances = [
            min(distance.euclidean((x, y), (rx, ry)) for rx, ry in zip(reference_x, reference_y))
            for x, y in zip(x_positions, y_positions)
        ]
        return {
            'mae': np.mean(distances),  # Mean Absolute Error
            'rmse': np.sqrt(np.mean(np.square(distances))),  # Root Mean Square Error
            'max_error': np.max(distances),  # Maximum Error
        }
    
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

    def calculate_total_operate_time(self):
        if len(self.start_times) == len(self.end_times):
            total_operate_time = sum(
                end - start for start, end in zip(self.start_times, self.end_times)
            )
            return total_operate_time
        return None
    
    def plot_trajectory(self):
        plt.figure(figsize=(8, 6))
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

        # Calculate error metrics
        leader_errors = self.calculate_errors(
            self.leader_x_positions, 
            self.leader_y_positions, 
            self.path_x_positions, 
            self.path_y_positions
        )
        follower_errors = self.calculate_errors(
            self.follower_x_positions, 
            self.follower_y_positions, 
            self.path_x_positions, 
            self.path_y_positions
        )

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Comparison of Planned and Actual Trajectories')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')

        # Displaying error metrics and total operating time
        # plt.text(0.05, 0.95, f"Operate time: {total_operate_time:.2f} seconds", 
        #          transform=plt.gca().transAxes, va='top')

        plt.show()

        # Also print error metrics and operating time in console
        print("Leader Errors:")
        print(f"  - MAE: {leader_errors['mae']:.2f}")
        print(f"  - RMSE: {leader_errors['rmse']:.2f}")
        print(f"  - Maximum Error: {leader_errors['max_error']:.2f}")

        print("Follower Errors:")
        print(f"  - MAE: {follower_errors['mae']:.2f}")
        print(f"  - RMSE: {follower_errors['rmse']:.2f}")
        print(f"  - Maximum Error: {follower_errors['max_error']:.2f}")

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



    def quaternion_to_yaw(self, quaternion):
        # Convert quaternion to yaw angle
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(q)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    planner = TrajectoryPlotter()

    def on_exit_signal_received(sig, frame):
        planner.plot_trajectory()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, on_exit_signal_received)

    rclpy.spin(planner)


if __name__ == '__main__':
    main()
