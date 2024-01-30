import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_vector3

class FormationController(Node):
    def __init__(self):
        super().__init__('formation_controller')
        self.robot1_frame = 'robot1/base_link'
        self.robot2_frame = 'robot2/base_link'
        self.target_distance = 1.0
        self.target_orientation = 0.0

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.control_callback)

    def control_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot1_frame, self.robot2_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f"Error looking up transform: {str(e)}")
            return

        # Calculate distance and orientation error
        distance_error = self.calculate_distance_error(transform)
        orientation_error = self.calculate_orientation_error(transform)

        # Implement your control algorithm here
        linear_velocity = 0.1 * distance_error
        angular_velocity = 0.1 * orientation_error

        # Publish control command
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_velocity
        cmd_msg.angular.z = angular_velocity
        self.publisher.publish(cmd_msg)

    def calculate_distance_error(self, transform):
        distance_vector = transform.transform.translation
        distance_error = self.target_distance - do_transform_vector3(distance_vector, transform).x
        return distance_error

    def calculate_orientation_error(self, transform):
        orientation_error = self.target_orientation - transform.transform.rotation.z
        return orientation_error

def main(args=None):
    rclpy.init(args=args)
    formation_controller = FormationController()
    rclpy.spin(formation_controller)
    formation_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
