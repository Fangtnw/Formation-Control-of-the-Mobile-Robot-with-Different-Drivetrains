import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
import geometry_msgs.msg 
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile
import tf_transformations

class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.camera_frame = 'camera_frame'  # Update with your camera frame
        self.aruco_frame = 'aruco_frame'    # Update with your ArUco frame
        self.camera_sub = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            qos_profile=QoSProfile(
                depth=1,  # This sets the history depth to 1 (optional)
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,  # Set reliability level
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,  # Optional, adjust if needed
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                lifespan=rclpy.duration.Duration(seconds=0.1) 
                  # Sets the deadline to 1/10 seconds for 10 Hz
            )
        )
        self.camera_matrix = np.array([
            
# [633.73045716,   0.0,         350.21328015],
#  [  0.0,         643.64092683, 300.82583886],
#  [  0.0,           0.0,           1.0        ]

    [593.99689349,   0.0         ,370.17478652],
    [  0.0,         596.41119583, 248.01189512],
    [  0.0,           0.0,           1.0,        ]


        # [509.186325 , 0.0, 345.35379696],
        # [  0.0 , 615.39364127, 306.27040277],
        # [  0.0 , 0.0 , 1.0        ]

            ])
        


# ([-0.44709562  0.36708006 -0.01366672 -0.00444425 -0.34651477])

        self.dist_coeff = np.array([-0.533307,    0.48569588, -0.00631618, -0.02238948, -0.38499538])
        # self.dist_coeff = np.array([-0.54810353, 0.89866936, 0.02231926, 0.01317876, -2.36918976])
        # self.dist_coeff = np.array([-0.44709562,  0.36708006, -0.01366672, -0.00444425, -0.34651477])

        


        # self.camera_info_sub = self.create_subscription(
        #     CameraInfo,
        #     'camera_info',
        #     self.camera_info_callback,
        #     qos_profile=QoSProfile(
        #         depth=10,  # This sets the history depth to 1 (optional)
        #         reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,  # Set reliability level
        #         durability=rclpy.qos.DurabilityPolicy.VOLATILE,  # Optional, adjust if needed
        #         history=rclpy.qos.HistoryPolicy.KEEP_LAST
        #           # Sets the deadline to 1/10 seconds for 10 Hz
        #     )
        # )
        self.tf_broadcaster = TransformBroadcaster(self)

    def image_callback(self, msg):
        self.get_logger().info("Received image message")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error("Error converting image: %s" % str(e))
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(
            gray, aruco_dict, parameters=parameters)

        if ids is not None:
            self.get_logger().info("Detected ArUco markers")
            for i, id in enumerate(ids):
                if id == 145:  # Example ArUco ID, update with your specific ID
                    self.get_logger().info("Found ArUco marker with ID 145")
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[i], 0.1, self.camera_matrix, self.dist_coeff)
                    
                    if rvec is not None and tvec is not None and tvec.any():  # Check if tvec is not empty
                        transform = TransformStamped()
                        transform.header.stamp = self.get_clock().now().to_msg()
                        transform.header.frame_id = self.aruco_frame
                        transform.child_frame_id = self.camera_frame
                        transform.transform.translation.y = -tvec[0][0][0]
                        transform.transform.translation.z = -tvec[0][0][1]
                        transform.transform.translation.x = tvec[0][0][2]
                        rotation_matrix = np.eye(4)
                        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvec[0][0]))[0]
                        r = R.from_matrix(rotation_matrix[0:3, 0:3])
                        quat = r.as_quat()  

                        # quat = tf_transformations.quaternion_from_euler(roll,pitch,yaw)
                        # Quaternion format     
                        transform.transform.rotation.y = quat[0] 
                        transform.transform.rotation.z = quat[1] 
                        transform.transform.rotation.x = -quat[2] 
                        transform.transform.rotation.w = quat[3] 
                        inverse_transform = self.invert_transform(transform)
                        self.get_logger().info("Publishing ArUco transform")
                        self.tf_broadcaster.sendTransform(transform)
                        break



    # def camera_info_callback(self, msg):
    #     self.get_logger().info("Received camera info message")
    #     self.camera_matrix = np.array(msg.k).reshape((3, 3))
    #     self.dist_coeff = np.array(msg.d)

    def invert_transform(self, transform):
        inverse_transform = TransformStamped()
        inverse_transform.header.stamp = self.get_clock().now().to_msg()
        inverse_transform.header.frame_id = transform.child_frame_id  # Swap the frames
        inverse_transform.child_frame_id = transform.header.frame_id

        translation = np.array([transform.transform.translation.x,
                                transform.transform.translation.y,
                                transform.transform.translation.z])
        
        quat = np.array([transform.transform.rotation.x,
                         transform.transform.rotation.y,
                         transform.transform.rotation.z,
                         transform.transform.rotation.w])

        # Inverse the rotation (conjugate of the quaternion)
        inverse_quat = np.array([quat[0], quat[1], quat[2], -quat[3]])
        inverse_rotation = R.from_quat(inverse_quat)

        # Inverse the translation
        inverse_translation = -inverse_rotation.apply(translation)

        # Assign the inverse translation and rotation to the transform
        inverse_transform.transform.translation.x = inverse_translation[0]
        inverse_transform.transform.translation.y = inverse_translation[1]
        inverse_transform.transform.translation.z = inverse_translation[2]

        inverse_quat = inverse_rotation.as_quat()
        inverse_transform.transform.rotation.x = inverse_quat[0]
        inverse_transform.transform.rotation.y = inverse_quat[1]
        inverse_transform.transform.rotation.z = inverse_quat[2]
        inverse_transform.transform.rotation.w = inverse_quat[3]

        return inverse_transform
    
def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArUcoDetector()

    # Use a MultiThreadedExecutor to allow callbacks to run concurrently
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(aruco_detector)
    try:
        executor.spin()
    finally:
        aruco_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
