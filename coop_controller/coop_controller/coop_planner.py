import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

class CubicPolynomialPlanner(Node):
    def __init__(self):
        super().__init__('cubic_polynomial_planner')  # Unique node name
        self.subscription = self.create_subscription(Odometry, '/diffdrive/odom', self.odom_callback, 10)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_local_plan = self.create_publisher(Path, '/plan', 10)
        self.via_points = [[1.5, -1.5],[4.0, -1.2],[5.0, -1.0], [5.0, 3.0]]
        self.fixed_via_points = [[1.5, -1.5],[4.0, -1.2],[5.0, -1.0], [5.0, 3.0]]  # Add additional via points
        self.tf = 1
        self.x_trajectory = []
        self.y_trajectory = []
        self.yaw_trajectory = []
        self.generate_trajectory(self.via_points, self.tf)

    def generate_trajectory(self, via_points, tf):
        t0 = 0.0
        dt = 0.1  # time step
        timesteps = int(tf / dt)

        # Interpolate the first segment from via_point 1 to via_point 3
        x_segment1, y_segment1 = self.interpolate_spline(via_points[:3], timesteps, tf/2)

        # Interpolate the second segment from via_point 3 to via_point 5
        x_segment2, y_segment2 = self.interpolate_spline(via_points[2:], timesteps, tf/2)

        # Combine the segments
        self.x_trajectory = x_segment1 + x_segment2  # Remove overlapping point
        self.y_trajectory = y_segment1 + y_segment2  # Remove overlapping point

        self.yaw_trajectory = [math.atan2(self.y_trajectory[i+1] - self.y_trajectory[i], self.x_trajectory[i+1] - self.x_trajectory[i])
                               for i in range(len(self.x_trajectory) - 1)]
        self.yaw_trajectory.append(self.yaw_trajectory[-1])

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
        # Store the current robot position as the first via point
        if not self.via_points:
            self.via_points.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
            self.via_points.extend(self.fixed_via_points)

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

def plot_trajectory(x_traj, y_traj, via_points):
    via_points = np.array(via_points)
    plt.figure(figsize=(8, 6))
    plt.plot(x_traj, y_traj, label='Trajectory', color='blue')
    plt.scatter(via_points[:, 0], via_points[:, 1], marker='o', color='red', label='Via Points')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Cubic Polynomial Spline Trajectory')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    planner = CubicPolynomialPlanner()
    planner.publish_trajectory()
    plot_trajectory(planner.x_trajectory, planner.y_trajectory, planner.via_points)
    rclpy.spin(planner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
