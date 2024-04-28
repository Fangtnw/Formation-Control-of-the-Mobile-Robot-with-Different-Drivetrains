import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import argparse
import signal
import pandas as pd
import os

class CubicPolynomialPlanner(Node):
    def __init__(self,path):
        super().__init__('cubic_polynomial_planner')  # Unique node name
        self.path = path
        self.subscription = self.create_subscription(Odometry, '/diffdrive/odom', self.odom_callback, 10)
        self.subscription_mec = self.create_subscription(Odometry, '/mec/odom', self.odom_callback_mec, 10)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_local_plan = self.create_publisher(Path, '/plan', 10)
        self.publisher_goal_pose = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.initial_pose_set = False  # Flag to check if initial pose is set
        self.initial_pose = None  # Store initial pose
        if self.path == '1.1':
            self.via_points = [[4.0, -1.2],[5.0, -1.0], [5.0, 4.1]]  #[1.5, -1.5]
        if self.path == '1.2':
            self.via_points = [[4.0, -1.7],[5.0, -1.0], [5.0, 4.1]]   #[1.5, -1.5]
        elif self.path == '2':
            self.via_points = [[1.5, -1.5],[8.0, -1.5],[6.0, -1.2],[5.0, -1.0], [5.0, 3.0]]
        elif self.path == '2.1':
            self.via_points = [[1.5, -1.5],[9.1, -1.5]]
        elif self.path == '2.2':
            self.via_points = [[6.0, -1.2],[5.0, -1.0], [5.0, 4.1]]
        elif self.path == '3.1':
            self.via_points = [[3.0, -1.7],[2.0, -1.9],[5.0, -1.0], [5.0, 3.0]]
        elif self.path == '3.2':
            self.via_points = [[5.0, -1.0], [5.0, 3.0]]

        self.tf = 1
        self.x_trajectory = []
        self.y_trajectory = []
        self.yaw_trajectory = []
        self.actual_leader_x_positions = []
        self.actual_leader_y_positions = []
        self.actual_follower_x_positions = []
        self.actual_follower_y_positions = []
        self.generate_trajectory(self.via_points, self.tf)
        
        # self.timer = self.create_timer(1.0, self.record_trajectories_to_csv)  # Timer fires every 1 second

    def record_trajectories_to_csv(self):
        # Ensure the directory for CSV files exists
        if self.latest_leader_x and self.latest_leader_y and self.latest_follower_x and self.actual_follower_y_positions is not None:
            output_dir = 'trajectory_data'
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)

            # Current timestamp for the filename
            timestamp = self.get_clock().now().to_msg().sec  # or use Time.now() in a different format

            # Filename for the CSV
            csv_filename = f'{output_dir}/trajectories_{self.path}_{timestamp}.csv'

            # Create a dictionary to record data
            data = {
                # 'x_trajectory': self.x_trajectory,
                # 'y_trajectory': self.y_trajectory,
                'leader_x_positions': self.latest_leader_x,
                'leader_y_positions': self.latest_leader_y,
                'follower_x_positions': self.latest_follower_x,
                'follower_y_positions': self.latest_follower_y,
            }

            # Create a DataFrame from the data
            df = pd.DataFrame(data)

            # Save to CSV
            df.to_csv(csv_filename, index=False)
            self.get_logger().info(f'Recorded trajectories to {csv_filename}')

    def generate_trajectory(self, via_points, tf):
        if self.initial_pose_set:
            t0 = 0.0
            dt = 0.01  # time step
            timesteps = int(tf / dt)

            # Interpolate the first segment from via_point 1 to via_point 3
            if self.path == '1.1':
                x_segment1, y_segment1 = self.interpolate_spline(via_points[:3], timesteps, tf/2)
                x_segment2, y_segment2 = self.interpolate_spline(via_points[2:], timesteps, tf/2)
                self.x_trajectory = x_segment1 + x_segment2  
                self.y_trajectory = y_segment1 + y_segment2  
            elif self.path == '1.2':
                x_segment1, y_segment1 = self.interpolate_spline(via_points[:3], timesteps, tf/2)
                x_segment2, y_segment2 = self.interpolate_spline(via_points[2:], timesteps, tf/2)
                self.x_trajectory = x_segment1 + x_segment2  
                self.y_trajectory = y_segment1 + y_segment2  
            elif self.path == '2':
                x_segment1, y_segment1 = self.interpolate_spline(via_points[:2], timesteps, tf/2)
                x_segment2, y_segment2 = self.interpolate_spline(via_points[1:4], timesteps, tf/2)
                x_segment3, y_segment3 = self.interpolate_spline(via_points[3:], timesteps, tf/2)
                self.x_trajectory = x_segment1 + x_segment2 + x_segment3  
                self.y_trajectory = y_segment1 + y_segment2 + y_segment3
            elif self.path == '2.1':
                x_segment1, y_segment1 = self.interpolate_spline(via_points, timesteps, tf/2)
                self.x_trajectory = x_segment1 
                self.y_trajectory = y_segment1 
            elif self.path == '2.2':
                x_segment1, y_segment1 = self.interpolate_spline(via_points[:3], timesteps, tf/2)
                x_segment2, y_segment2 = self.interpolate_spline(via_points[2:], timesteps, tf/2)
                self.x_trajectory = x_segment1 + x_segment2  
                self.y_trajectory = y_segment1 + y_segment2  
            elif self.path == '3.1':
                x_segment1, y_segment1 = self.interpolate_spline(via_points[:2], timesteps, tf/2)
                x_segment2, y_segment2 = self.interpolate_spline(via_points[1:3], timesteps, tf/2)
                x_segment3, y_segment3 = self.interpolate_spline(via_points[2:4], timesteps, tf/2)
                x_segment4, y_segment4 = self.interpolate_spline(via_points[3:], timesteps, tf/2)
                self.x_trajectory = x_segment1 + x_segment2 + x_segment3 + x_segment4 
                self.y_trajectory = y_segment1 + y_segment2 + y_segment3 + y_segment4
            elif self.path == '3.2':
                x_segment1, y_segment1 = self.interpolate_spline(via_points[:2], timesteps, tf/2)
                x_segment2, y_segment2 = self.interpolate_spline(via_points[1:], timesteps, tf/2)
                self.x_trajectory = x_segment1 + x_segment2  
                self.y_trajectory = y_segment1 + y_segment2  

            # Combine the segments

            yaw_trajectory = [
                math.atan2(self.y_trajectory[i + 1] - self.y_trajectory[i], self.x_trajectory[i + 1] - self.x_trajectory[i])
                for i in range(len(self.x_trajectory) - 1)
            ]
            start_index_segment2 = len(x_segment1) - 1  # Segment 2 starts at the end of segment 1
            end_index_segment2 = start_index_segment2 + len(x_segment2)
            
            # Invert the yaw angles for segment 2
            # for i in range(start_index_segment2, end_index_segment2):
            #     yaw_trajectory[i] = -yaw_trajectory[i]  # Invert the yaw angle for this segment

            for i in range(start_index_segment2, end_index_segment2):
                yaw_trajectory[i] = (yaw_trajectory[i] + math.pi)  % (2 * math.pi)           
            self.yaw_trajectory = yaw_trajectory
            # if self.path in ['1,1','3.1', '3.2']:
            #     self.yaw_trajectory = [(yaw + math.pi) % (2 * math.pi) for yaw in self.yaw_trajectory]


            if self.yaw_trajectory:
                self.yaw_trajectory.append(self.yaw_trajectory[-1])
        else:
            pass

    def interpolate_spline(self, via_points, timesteps, tf):
        num_points = len(via_points)
        x_points = [point[0] for point in via_points]
        y_points = [point[1] for point in via_points]

        t = np.linspace(0, 1, num_points)
        sx = CubicSpline(t, x_points)
        sy = CubicSpline(t, y_points)

        t_interpolate = np.linspace(0, 1, timesteps * (num_points - 1))
        x_interpolated = sx(t_interpolate)
        y_interpolated = sy(t_interpolate)

        return x_interpolated.tolist(), y_interpolated.tolist()

    def odom_callback(self, msg):
        if not self.initial_pose_set:
            # Store the initial pose of the robot as the first via point
            self.via_points.insert(0, [msg.pose.pose.position.x, msg.pose.pose.position.y])
            self.initial_pose_set = True
            self.initial_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
            self.get_logger().info("get initial pose")
        else:
            # Replace the initial via point with the updated pose
            # self.via_points[0] = [msg.pose.pose.position.x, msg.pose.pose.position.y]
            # If initial pose is set, generate trajectory and publish it
            self.generate_trajectory(self.via_points, self.tf)
            self.publish_trajectory()
            
            # self.plot_trajectory()
        self.actual_leader_x_positions.append(msg.pose.pose.position.x)
        self.actual_leader_y_positions.append(msg.pose.pose.position.y)
        self.latest_leader_x = msg.pose.pose.position.x
        self.latest_leader_y = msg.pose.pose.position.y
            
    def odom_callback_mec(self, msg):
        self.actual_follower_x_positions.append(msg.pose.pose.position.x)
        self.actual_follower_y_positions.append(msg.pose.pose.position.y)
        self.latest_follower_x = msg.pose.pose.position.x
        self.latest_follower_y = msg.pose.pose.position.y

    def publish_goal_pose(self):
        if not self.x_trajectory or not self.y_trajectory:
            self.get_logger().warn("No trajectory data available to set goal.")
            return
        
        # Get the final point in the trajectory
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map'

        # The goal is the last point in the trajectory
        goal_pose.pose.position.x = self.x_trajectory[-1]
        goal_pose.pose.position.y = self.y_trajectory[-1]

        # Using the last yaw from the trajectory
        yaw = self.yaw_trajectory[-1]
        quaternion = self.yaw_to_quaternion(yaw)
        
        goal_pose.pose.orientation.x = quaternion[0]
        goal_pose.pose.orientation.y = quaternion[1]
        goal_pose.pose.orientation.z = quaternion[2]
        goal_pose.pose.orientation.w = quaternion[3]

        self.publisher_goal_pose.publish(goal_pose)
        self.get_logger().info(f"Published goal pose at x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}")

    def publish_trajectory(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for x, y, yaw in zip(self.x_trajectory, self.y_trajectory, self.yaw_trajectory):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            quaternion = self.yaw_to_quaternion(yaw)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            path_msg.poses.append(pose)

        self.publisher_local_plan.publish(path_msg)

    def yaw_to_quaternion(self, yaw):
        quaternion = [0.0, 0.0, 0.0, 0.0]
        quaternion[0] = 0.0
        quaternion[1] = 0.0
        quaternion[2] = math.sin(yaw / 2)
        quaternion[3] = math.cos(yaw / 2)
        return quaternion

    def plot_trajectory(self):
        # Convert via_points to a NumPy array
        via_points_np = np.array(self.via_points)
        
        # Correct plotting logic
        plt.figure(figsize=(8, 6))
        plt.plot(self.x_trajectory, self.y_trajectory, label='Planned Trajectory', color='blue')
        plt.plot(self.actual_leader_x_positions, self.actual_leader_y_positions, label='Leader Trajectory', color='orange')
        plt.plot(self.actual_follower_x_positions, self.actual_follower_y_positions, label='Follower Trajectory', color='green')
        
        # Using NumPy array, now you can use tuple-based indexing
        plt.scatter(via_points_np[:, 0], via_points_np[:, 1], marker='o', color='red', label='Via Points')

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Comparison of Planned and Actual Trajectories')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Cubic_Polynomial_Planner')
    parser.add_argument('--path', choices=['1.1','1.2','2','2.1','2.2','3.1','3.2','4'], required=True,
                        help='Specify the desired path')
    args = parser.parse_args()
    planner = CubicPolynomialPlanner(args.path)
    # planner.publish_trajectory()
    # signal.signal(signal.SIGINT, on_exit_signal_received)

    def on_exit_signal_received(sig, frame):
        # Handle Ctrl+C interrupt
        # planner.publish_goal_pose()
        planner.plot_trajectory()  # Plot trajectory before shutting down
        rclpy.shutdown()

    signal.signal(signal.SIGINT, on_exit_signal_received)

    rclpy.spin(planner)

if __name__ == '__main__':
    main()
