import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
import math

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
        self.target_speed = 0.2  # Set your desired speed here
        self.lookahead_distance = 0.5  # Set the lookahead distance here
        self.kp = 0.5  # Proportional control gain

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
        total_distance = 0
        while total_distance < self.lookahead_distance and lookahead_index < len(self.waypoints) - 1:
            lookahead_index += 1
            total_distance += self.distance_between_points(
                self.waypoints[lookahead_index].pose.position,
                self.waypoints[lookahead_index - 1].pose.position
            )

        # Calculate target point coordinates
        target_x = self.waypoints[lookahead_index].pose.position.x
        target_y = self.waypoints[lookahead_index].pose.position.y

        # Calculate errors
        delta_x = target_x - self.current_pose.position.x
        delta_y = target_y - self.current_pose.position.y
        distance_error = math.sqrt(delta_x ** 2 + delta_y ** 2)

        # Calculate desired heading angle
        desired_heading = math.atan2(delta_y, delta_x)

        # Calculate steering angle (pure pursuit controller)
        steering_angle = math.atan2(2.0 * 0.1 * delta_y, distance_error)

        # Calculate linear velocity
        linear_velocity = -min(self.target_speed, self.kp * distance_error)

        # Publish control commands
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = steering_angle
        self.publisher_cmd_vel.publish(cmd_vel_msg)

        self.get_logger().info("Control commands published: Linear Velocity = {}, Steering Angle = {}".format(linear_velocity, steering_angle))

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
