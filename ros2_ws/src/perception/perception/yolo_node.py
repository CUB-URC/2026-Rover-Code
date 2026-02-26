import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge

import cv2
import numpy as np

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False


class YoloNode(Node):
    def __init__(self):
        super().__init__("yolo_node")

        # --- STATE ---
        self.model = None # one at a time
        self.bridge = CvBridge() # convert ROS img <-> CV format

        self._setup_parameters()
        self._setup_subscriptions()
        self._setup_publishers()
        self._load_model()
    
    def _setup_parameters(self):
        self.model_path = os.path.expanduser(self.declare_parameter("model_path", "").get_parameter_value().string_value)
        self.confidence_threshold = self.declare_parameter("confidence_threshold", 0.5).get_parameter_value().double_value
        self.class_names = self.declare_parameter("class_names", ["hammer", "mallet", "bottle"]).get_parameter_value().string_array_value

    def _load_model(self):
        if not ULTRALYTICS_AVAILABLE:
            self.get_logger().warn("ultralytics not installed — YOLO inference disabled. run: pip install ultralytics")
            return
        
        if not self.model_path:
            self.get_logger().warn("no model_path set — YOLO inference disabled. pass via: -p model_path:=/abs/path/model.onnx")
            return
        
        if not os.path.exists(self.model_path):
            self.get_logger().warn(f"model file not found: {self.model_path} — YOLO inference disabled")
            return
        
        self.model = YOLO(self.model_path)
        self.get_logger().info(f"YOLO model loaded from: {self.model_path}")

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
            "/image", # remap at launch
            self.listener_callback,
            qos_profile=qos
        )

    def _setup_publishers(self):
        
        self.debug_pub = self.create_publisher(Image, "perception/yolo_debug", 10)

        self.detections_pub = self.create_publisher(
            Detection2DArray,
            "perception/detections",
            10
        )

        # publish highest confidence detection to  mission/object_detection (req: only highlight one object)
        self.best_detection_pub = self.create_publisher(
            Detection2D,
            "mission/object_detection",
            10
        )
    
    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 1. run inference on model (passthrough debug image if no model loaded)
            if self.model is None:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                return
            results = self.model(cv_image, conf=self.confidence_threshold, verbose=False)
            res = results[0]

            # 2. create the detection image array
            detection_arr = Detection2DArray()
            detection_arr.header.stamp = msg.header.stamp
            detection_arr.header.frame_id = msg.header.frame_id

            best_detection = None
            best_confidence = 0.0

            # 3. loop thru bounding boxes in the result
            for box in res.boxes:
                # 3.1 set the current confidence level and class params
                conf = float(box.conf[0]) # 1-element tensor
                cls_idx = int(box.cls[0])
                cls_name = (
                    self.class_names[cls_idx]
                    if cls_idx < len(self.class_names)
                    else str(cls_idx)
                )

                # 3.2 extract coordinates and calculate center, width, height
                x1, y1, x2, y2 = map(int, box.xyxy[0]) # bounding box pixel coordinates
            
                cx = float((x1 + x2) / 2)
                cy = float((y1 + y2) / 2)
                w = float(x2 - x1)
                h = float(y2 - y1)

                # 3.3 create detection image, set bbox params
                det = Detection2D() 
                det.header.stamp = msg.header.stamp
                det.bbox.center.position.x = cx
                det.bbox.center.position.y = cy
                det.bbox.size_x = w
                det.bbox.size_y = h
                
                # 3.4 create object prediction, set class id and score to current class name and confidence lvl
                pred = ObjectHypothesisWithPose()
                pred.hypothesis.class_id = cls_name
                pred.hypothesis.score = conf
                det.results.append(pred)  # append before adding det to array

                detection_arr.detections.append(det)

                # 3.5 update best detection if current confidence is greater than previous
                if conf > best_confidence:
                    best_confidence = conf
                    best_detection = det

                # 3.6 draw on debug image
                color = (0, 255, 0)
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    cv_image,
                    f"{cls_name} {conf:.2f}",
                    (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, color, 2
                )

            # 4. publish detections array
            self.detections_pub.publish(detection_arr)

            # 5. publish best detection if found
            if best_detection is not None:
                self.best_detection_pub.publish(best_detection)
                self.get_logger().info(
                    f"Best detection: {best_detection.results[0].hypothesis.class_id}"
                    f"({best_confidence:.3f})"
                )

            # 6. publish debug image
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        except Exception as e:
            self.get_logger().error(f"Error in YOLO listener_callback: {str(e)}")
        

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
