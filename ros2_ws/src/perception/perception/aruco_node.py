"""
////////////////////////////////////////////////////////////////
                    ARUCO TAG IDENTIFICATION
////////////////////////////////////////////////////////////////


[x] establish ZED 2i driver communication
[x] implement real-time 2D ArUco tag identification
[x] configure QoS for low-latency video streaming
[ ] integrate zed depth map to get 3D (X, Y, Z) tag coordinates
[ ] map tag ids to specific mission objectives
[ ] fuse with YOLO model for obstacle avoidance


////////////////////////////////////////////////////////////////

for testing U need...

-   a system with CUDA support (nvidia gpu)
-   you need the zed sdk.
-   you need the zed-ros2-wrapper and zed-ros2-interfaces repos. 
    these must be in the /src folder with the other packages


run these two commands in each of the three terminals youre gonna need for sourcing

    source /opt/ros/humble/setup.bash
    source ~/2026-Rover-Code/ros2_ws/install/setup.bash

rebuild everything via

    colcon build --symlink-install

terminal 1: start cam driver

    ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i

    [faster]

        ros2 launch zed_wrapper zed_camera.launch.py \
        camera_model:=zed2i \
        general.video_resolution:=VGA \
        general.pub_frame_rate:=10.0 \
        depth.depth_mode:=NONE

terminal 2: start this node
    
    # remap the generic /image input to what the zed 2i likes
    ros2 run perception aruco_node --ros-args --remap /image:=/zed/zed_node/rgb/color/rect/image

terminal 3: visualization
    
    ros2 run rqt_image_view rqt_image_view
    # select '/perception/aruco_debug' in the top-left dropdown

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray
from cv_bridge import CvBridge

import cv2
import numpy as np


class ArucoNode(Node):
    def __init__(self):
        super().__init__("aruco_node")

        # --- PARAMETERS ---
        self.declare_parameter("aruco_dictionary_id", "DICT_4X4_50")
        self.dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )

        # --- QOS PROFILE ---
        # ZED uses "Best Effort" (UDP ish) transmission. we must match it
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        """ TODO
        [] from camera calibration, get the intrinsic matrix and distortion coefficients
        """

        # --- PARAMETERS ---
        self.declare_parameter("intrinsic_matrix", [])  # intrinsic matrix
        self.declare_parameter("distortion_coeffs", [])  # distortion coefficients
        self.declare_parameter(
            "marker_length", 0.02
        )  # default to 2 cm (adjust as needed) -> for pose estimation

        self.intrinsic_matrix = None
        self.distortion_coeffs = None

        # --- SUBSCRIBERS ---
        self.subscription = self.create_subscription(
            Image,
            "/image",  # remap ts
            self.listener_callback,
            qos_profile=qos_profile,  # qos profile Duh
        )

        # sub to camera info to get calibration data for pose estimation
        self.camera_info_sub = self.create_subscription(
            CameraInfo,  # todo
            "/zed/zed_node/rgb/camera_info",
            self.camera_info_callback,
            qos_profile=qos_profile,
        )

        # --- PUBLISHERS ---
        self.publisher_ = self.create_publisher(Image, "perception/aruco_debug", 10)

        self.pose_publisher = self.create_publisher(
            PoseArray, "perception/aruco_poses", 10  # todo  # ?
        )

        # --- SETUP ---
        self.bridge = CvBridge()

        try:
            dictionary_id = getattr(cv2.aruco, self.dictionary_id_name)
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.get_logger().info(
                f"Loaded ArUco Dictionary: {self.dictionary_id_name}"
            )
        except AttributeError:
            self.get_logger().error(
                f"Invalid ArUco Dictionary: {self.dictionary_id_name}"
            )

    def camera_info_callback(self, msg):
        """Get camera calibration from ZED camera_info topic for pose estimation"""
        if self.intrinsic_matrix is None:
            self.intrinsic_matrix = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d)
            self.get_logger().info("Received camera calibration parameters")

    def listener_callback(self, msg):
        try:
            # 1. convert ROS image -> OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 2. detect markers
            corners, ids, rejected = cv2.aruco.detectMarkers(
                cv_image, self.aruco_dict, parameters=self.aruco_params
            )

            # 3. if found, print IDs and draw boxes
            if ids is not None and self.intrinsic_matrix is not None:
                id_list = ids.flatten()
                self.get_logger().info(f"Tags Detected: {id_list}")
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

                # 3.1 estimate pose
                pose_array = PoseArray()

            # 4. publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.publisher_.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
