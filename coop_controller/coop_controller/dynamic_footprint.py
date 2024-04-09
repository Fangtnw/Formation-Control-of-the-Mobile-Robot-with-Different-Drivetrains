import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Polygon, Point32
import argparse
import tf_transformations
from tf2_ros import Buffer, TransformListener

class DynamicFootprint(Node):

    def __init__(self, follower_type):
        super().__init__('dynamic_footprint')
        self.follower_type = follower_type
        # Follower robot frame ID (replace with the actual frame ID)
        if self.follower_type == 'mecanum':
            self.robot1_frame = 'base_footprint'
            self.robot2_frame = 'base_footprint_mec'
        elif self.follower_type == 'diffdrive':
            self.robot1_frame = 'base_footprint'
            self.robot2_frame = 'base_footprint_diff'

        # Original footprint points (modify these as needed)
        self.original_footprint = [[1.90, 0.3], [1.25, 0.3], [1.25, 0.17], [0.0, 0.17],
                                   [0.0, 0.3], [-0.50, 0.3], [-0.50, -0.3], [0.0, -0.3],
                                   [0.0, -0.17], [1.25, -0.17], [1.25, -0.3], [1.90, -0.3]]

        # Footprint publisher (replace with the appropriate topic)
        self.footprint_pub = self.create_publisher(Polygon, '/local_costmap/footprint', 10)

        # Follower pose subscriber
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Timer to publish footprint at a regular rate (adjust as needed)
        self.footprint_timer = self.create_timer(0.5, self.publish_footprint)

    def publish_footprint(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot1_frame, self.robot2_frame, rclpy.time.Time())
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z
            x = transform.transform.rotation.x
            y = transform.transform.rotation.y
            w = transform.transform.rotation.w
            z = transform.transform.rotation.z
            euler = tf_transformations.euler_from_quaternion([x, y, w, z])
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]
            if self.follower_type == 'mecanum':
                dx = (tx - 1.65)
            elif self.follower_type == 'diffdrive':
                dx = (tx - 1.45)
            # self.get_logger().info(
            #     f"Translation: x:{tx:.2f}, y:{ty:.2f}, z:{tz:.2f} , "
            #     f"Rotation: roll:{roll:.2f}, pitch:{pitch:.2f}, yaw:{yaw:.4f}"
            # )

            footprint_msg = Polygon()
            footprint_msg.points = [
                Point32(x=tx+0.275, y=ty+0.3, z=0.0),
                Point32(x=tx-0.275, y=ty+0.3, z=0.0),
                Point32(x=1.25, y=0.17, z=0.0),
                Point32(x=0.0, y=0.17, z=0.0),
                Point32(x=0.0, y=0.3, z=0.0),
                Point32(x=-0.50, y=0.3, z=0.0),
                Point32(x=-0.50, y=-0.3, z=0.0),
                Point32(x=0.0, y=-0.3, z=0.0),
                Point32(x=0.0, y=-0.17, z=0.0),
                Point32(x=1.25, y=-0.17, z=0.0),

                Point32(x=tx-0.275, y=ty-0.3, z=0.0),
                Point32(x=tx+0.275, y=ty-0.3, z=0.0)
            ]

            self.footprint_pub.publish(footprint_msg)

        except Exception as e:
            self.get_logger().error(f"Error looking up transform: {str(e)}")
            return

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='DynamicFootprint')
    parser.add_argument('--follower_type', choices=['mecanum', 'diffdrive', 'aruco'], required=True,
                        help='Specify follower robot type (mecanum or diffdrive)')
    args = parser.parse_args()
    node = DynamicFootprint(args.follower_type)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
