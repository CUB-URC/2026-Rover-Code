import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, TransformStamped
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster # for RViz

import cv2
import numpy as np

class ArucoNode(Node):
    def __init__(self):
        super().__init__("aruco_node")

        # --- LOOKUP TABLES ---
        self.marker_sizes = {}
        self.marker_missions = {}

        # --- STATE INIT ---
        self.intrinsic_matrix = None
        self.distortion_coeffs = None
        self.aruco_dict = None
        self.aruco_params = None
        self.aruco_detector = None  # ArucoDetector object (OpenCV 4.7+)
        # TODO: latest depth // if using depth refinement

        # --- SETUP ---
        self.bridge = CvBridge()

        self._setup_parameters()  
        self._load_config()         
        self._setup_subscriptions()
        self._setup_publishers()

        try:
            dictionary_id = getattr(cv2.aruco, self.dictionary_id_name)
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
            # OpenCV 4.7+ uses ArucoDetector; 4.5.x uses detectMarkers + DetectorParameters
            if hasattr(cv2.aruco, "ArucoDetector"):
                params = cv2.aruco.DetectorParameters()
                self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, params)
                self.aruco_params = None  # not used in new API
            else:
                self.aruco_detector = None  # use legacy detectMarkers path
                self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.get_logger().info(
                f"Loaded ArUco Dictionary: {self.dictionary_id_name}"
            )
        except Exception as e:
            self.get_logger().fatal(
                f"Failed to load ArUco dictionary '{self.dictionary_id_name}': {e}. "
                "Check that the dictionary name is valid (e.g. DICT_4X4_50)."
            )
            raise

    def _load_config(self):
        """maps marker ids -> mission objectives"""

        for mission, details in self.config['marker_mappings'].items():
            for id in details['ids']:
                self.marker_sizes[id] = details['size']
                self.marker_missions[id] = mission

        self.get_logger().info(f"Loaded config: {self.marker_missions}")

    def _setup_parameters(self):
        # --- PARAMETERS ---
        config_path = self.declare_parameter("config_path", "config/aruco_config.yaml").get_parameter_value().string_value
        try:
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().fatal(
                f"Config file not found: '{config_path}'. "
                "Pass the absolute path via: --ros-args -p config_path:=/absolute/path/to/aruco_config.yaml"
            )
            raise
        config = self.config 
        
        self.declare_parameter("aruco_dictionary_id", config.get('aruco_dictionary_id', 'DICT_4X4_50'))
        self.dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )


        self.declare_parameter("intrinsic_matrix", [])  
        self.declare_parameter("distortion_coeffs", []) 
        self.declare_parameter(
            "marker_length", 0.02
        )  # default to 2 cm (adjust as needed) -> for pose estimation

        # make this topic remappable so webcam testing works:
        #   --ros-args -p camera_info_topic:=/camera_info
        self.camera_info_topic = self.declare_parameter(
            "camera_info_topic", "/zed/zed_node/rgb/color/rect/camera_info"
        ).get_parameter_value().string_value

    def _setup_subscriptions(self):

          # --- QOS PROFILE ---
        # ZED uses "Best Effort" (UDP ish) transmission. we must match it
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.subscription = self.create_subscription(
            Image,
            "/image",  # remap ts
            self.listener_callback,
            qos_profile=qos
        )

        # sub to camera info to get calibration data for pose estimation
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile=qos,
        )

    def _setup_publishers(self):
        
        self.publisher_ = self.create_publisher(Image, "perception/aruco_debug", 10)

        self.pose_publisher = self.create_publisher(
            PoseArray, "perception/aruco_poses", 10  # TODO - adjust topic name ? id
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        # publish: per mission pubs (keyboard, usb, nav posts)        
        self.keyboard_pub = self.create_publisher(PoseStamped, "mission/keyboard_pose", 10)
        self.usb_pub = self.create_publisher(PoseStamped, "mission/usb_pose", 10)
        self.nav_post_1_pub = self.create_publisher(PoseStamped, "mission/nav_post_1_pose", 10)
        self.nav_post_2_pub = self.create_publisher(PoseStamped, "mission/nav_post_2_pose", 10)
        self.start_gate_pub = self.create_publisher(PoseStamped, "mission/start_gate_pose", 10)

        # TODO: TF broadcast publisher for rviz

    def camera_info_callback(self, msg):
        """Get camera calibration from camera_info topic for pose estimation.
        Rejects the message if K is all zeros (usb_cam publishes this when
        no calibration file exists).
        """
        if self.intrinsic_matrix is None:
            k = np.array(msg.k).reshape(3, 3)
            if k[0, 0] == 0.0:
                self.get_logger().warn(
                    "camera_info received but K matrix is all zeros — "
                    "no calibration file loaded. Detection will work but "
                    "pose estimation is disabled until calibration is available.",
                    throttle_duration_sec=5.0
                )
                return
            self.intrinsic_matrix = k
            self.distortion_coeffs = np.array(msg.d)
            self.get_logger().info("Received camera calibration parameters — pose estimation enabled")

    def listener_callback(self, msg):
        try:
            if self.aruco_dict is None and self.aruco_detector is None:
                self.get_logger().error(
                    "ArUco detector not initialized — check startup logs.",
                    throttle_duration_sec=5.0
                )
                return

            # 1. convert ROS image -> OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 2. detect markers
            if self.aruco_detector is not None:
                # OpenCV 4.7+ ArucoDetector API
                corners, ids, _ = self.aruco_detector.detectMarkers(cv_image)
            else:
                # OpenCV 4.5.x legacy API
                corners, ids, _ = cv2.aruco.detectMarkers(
                    cv_image, self.aruco_dict, parameters=self.aruco_params
                )

            # 3. always draw detections when markers are found
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

                # 3.1 pose estimation — only when calibration is available
                if self.intrinsic_matrix is not None:
                    pose_array = PoseArray()
                    pose_array.header.stamp = msg.header.stamp
                    pose_array.header.frame_id = "zed_camera_frame"  # adjust

                    for i, corner in enumerate(corners):
                        marker_id = ids[i][0]
                        marker_size = self.marker_sizes.get(marker_id, 0.02)

                        # 3.2 estimate pose of each marker
                        # estimatePoseSingleMarkers removed in OpenCV 4.7+ — use solvePnP directly.
                        # define the marker's 4 corners in its own local 3D space (Z=0 plane)
                        half = marker_size / 2.0
                        obj_points = np.array([
                            [-half,  half, 0],
                            [ half,  half, 0],
                            [ half, -half, 0],
                            [-half, -half, 0],
                        ], dtype=np.float32)
                        img_points = corner[0].astype(np.float32)  # shape (4,2)
                        _, rvec, tvec = cv2.solvePnP(
                            obj_points, img_points,
                            self.intrinsic_matrix, self.distortion_coeffs
                        )
                        # reshape to match old estimatePoseSingleMarkers output shape
                        rvec = rvec.reshape(1, 1, 3)
                        tvec = tvec.reshape(1, 1, 3)

                        # todo: integrate depth

                        # 3.3-4 build the pose from tvec and quat from rotation matrix
                        pose, quat = self._build_pose(tvec, rvec)
                       
                         # 3.5 draw axis on image
                        cv2.drawFrameAxes(cv_image, self.intrinsic_matrix, self.distortion_coeffs, rvec, tvec, marker_size * 0.5)

                        pose_array.poses.append(pose)
                        self._broadcast_marker_tf(marker_id, pose, quat, msg.header.stamp)

                        # 3.6 publish mission-specific topic for this marker
                        self._publish_mission_objective(marker_id, pose, msg.header.stamp)

                        self.get_logger().info(
                            f"Marker {marker_id}: pose=({tvec[0][0][0]:.3f}, "
                            f"{tvec[0][0][1]:.3f}, {tvec[0][0][2]:.3f})"
                        )

                    # 3.7 publish pose array
                    self.pose_publisher.publish(pose_array)

                else:
                    # detection-only mode: log IDs, no pose output
                    missions = [self.marker_missions.get(marker_id, "unknown") for marker_id in ids.flatten()]
                    
                    self.get_logger().info(
                        f"Tags Detected (no calibration — detection only): {ids.flatten()} | Missions: {missions}",
                        throttle_duration_sec=2.0
                    )

            # 4. publish debug image (always, regardless of detections or calibration)
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.publisher_.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def _build_pose(self, tvec, rvec):
        # 1. convert to ROS pose message
        pose = Pose()
        pose.position.x = tvec[0][0][0]
        pose.position.y = tvec[0][0][1]
        pose.position.z = tvec[0][0][2]

        # 2. convert rotation vector to quaternion
        rotation_matrix = cv2.Rodrigues(rvec[0][0])[0]
        quat = R.from_matrix(rotation_matrix).as_quat()  # [x, y, z, w]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        return pose, quat

    def _publish_mission_objective(self, marker_id, pose, timestamp):
        """Given a detected marker ID, publish the corresponding mission objective"""
        mission = self.marker_missions.get(marker_id, "unknown")

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = timestamp
        pose_stamped.header.frame_id = "zed_camera_frame"  # adjust as needed
        pose_stamped.pose = pose

        pub_map = {
            "keyboard": self.keyboard_pub,
            "usb_slot": self.usb_pub,
            "nav_post_1": self.nav_post_1_pub,
            "nav_post_2": self.nav_post_2_pub,
            "start_gate": self.start_gate_pub,
        }

        pub = pub_map.get(mission)
        if pub is not None:
            pub.publish(pose_stamped)
        else: 
            self.get_logger().warn(f"No publisher found for mission: {mission}")

        self.get_logger().info(f"Marker {marker_id} corresponds to mission: {mission}")

    def _broadcast_marker_tf(self, marker_id, pose, quat, timestamp):
        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = timestamp
        tf_stamped.header.frame_id = "zed_camera_frame"
        tf_stamped.child_frame_id = f"aruco_marker_{marker_id}"

        tf_stamped.transform.translation.x = pose.position.x
        tf_stamped.transform.translation.y = pose.position.y
        tf_stamped.transform.translation.z = pose.position.z

        tf_stamped.transform.rotation.x = quat[0]
        tf_stamped.transform.rotation.y = quat[1]
        tf_stamped.transform.rotation.z = quat[2]
        tf_stamped.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(tf_stamped)


    # TODO: depth refine callback and depth callback (???)


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
