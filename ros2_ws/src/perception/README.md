# Perception

Perception package for the CUB-URC rover. Handles ArUco tag detection/pose estimation and YOLO object detection.

---

## Setup

### Dependencies

Install Python deps:
```bash
pip install -r ros2_ws/src/perception/requirements.txt
```

> **⚠️ NumPy pin:** `numpy<2` is required. `ultralytics` will try to upgrade NumPy to 2.x, which breaks `cv_bridge`.

Install ROS2 apt deps (if missing):
```bash
sudo apt install ros-humble-usb-cam ros-humble-vision-msgs
```

### Build

```bash
cd ~/2026-Rover-Code/ros2_ws
colcon build --packages-select perception
```

### Source

```bash
source /opt/ros/humble/setup.bash && \
source $HOME/2026-Rover-Code/ros2_ws/install/setup.bash
```

---

## Nodes

### `aruco_node`

Detects ArUco markers in a camera image, estimates their 6-DOF pose, broadcasts TF frames, and publishes mission-specific pose topics.

**Subscribes to:**
| Topic | Type | Notes |
|---|---|---|
| `/image` | `sensor_msgs/Image` | remap at launch |
| `camera_info_topic` (param) | `sensor_msgs/CameraInfo` | defaults to ZED topic; override with `-p` for webcam |

**Publishes to:**
| Topic | Type | Notes |
|---|---|---|
| `perception/aruco_debug` | `sensor_msgs/Image` | annotated debug image |
| `perception/aruco_poses` | `geometry_msgs/PoseArray` | all detected marker poses |
| `mission/keyboard_pose` | `geometry_msgs/PoseStamped` | equipment servicing |
| `mission/usb_pose` | `geometry_msgs/PoseStamped` | equipment servicing |
| `mission/nav_post_1_pose` | `geometry_msgs/PoseStamped` | autonomous navigation |
| `mission/nav_post_2_pose` | `geometry_msgs/PoseStamped` | autonomous navigation |
| `mission/start_gate_pose` | `geometry_msgs/PoseStamped` | autonomous navigation |
| TF frames | — | `aruco_marker_<id>` child frames under `camera` |

**Parameters:**
| Parameter | Default | Notes |
|---|---|---|
| `config_path` | — | **required** — absolute path to `aruco_config.yaml` |
| `camera_info_topic` | `/zed/zed_node/rgb/color/rect/camera_info` | override for webcam |
| `aruco_dictionary_id` | `DICT_4X4_50` | must match printed markers |

**Available Modes:**
- **Detection-only (testing)** (no calibration): draws bounding boxes, logs IDs — no pose math. This is what you get with a fresh uncalibrated webcam.
- **Full pose estimation**: activates once a valid `CameraInfo` message is received (non-zero K matrix).

**Marker ID → mission mapping** lives in `config/aruco_config.yaml`; update here if IDs change.

---

### `yolo_node`

Runs live YOLO inference on a camera image stream. Detects the 3 required objects (hammer, mallet, bottle) and publishes bounding boxes.

**Subscribes to:**
| Topic | Type | Notes |
|---|---|---|
| `/image` | `sensor_msgs/Image` | remap at launch |

**Publishes to:**
| Topic | Type | Notes |
|---|---|---|
| `perception/yolo_debug` | `sensor_msgs/Image` | annotated debug image |
| `perception/detections` | `vision_msgs/Detection2DArray` | all detections in this frame |
| `mission/object_detection` | `vision_msgs/Detection2D` | highest-confidence detection only |

**Parameters:**
| Parameter | Default | Notes |
|---|---|---|
| `model_path` | — | **required** — absolute path to `.onnx` or `.pt` model |
| `confidence_threshold` | `0.5` |  |
| `class_names` | `["hammer", "mallet", "bottle"]` | must match order in Roboflow `data.yaml` |

**Models** live in `perception/models/<class_name>/best.onnx`. Export from Roboflow as ONNX for Jetson deployment (avoids full PyTorch).

| Model | Status |
|---|---|
| `models/hammer/best.onnx` | ✅ exists |
| `models/mallet/best.onnx` | ⬜ TODO: train / export |
| `models/bottle/best.onnx` | ⬜ TODO: train / export |

---

## Testing

### Webcam Only (dev)

#### Webcam Calibration (for pose estimation)

```bash
sudo apt install ros-humble-camera-calibration

# terminal 1
ros2 run usb_cam usb_cam_node_exe --ros-args \
    -p video_device:=/dev/video0 \
    -p framerate:=30.0 \
    -p image_width:=640 \
    -p image_height:=480

# terminal 2
ros2 run camera_calibration cameracalibrator \
    --size 8x6 \
    --square 0.024 \
    --ros-args --remap /image:=/image_raw
```

Move the checkerboard around until all bars turn green, click **Calibrate**, then **Save**. Extract the result:
```bash
cd /tmp && tar -xzf calibrationdata.tar.gz
mkdir -p ~/.ros/camera_info
cp ost.yaml ~/.ros/camera_info/default_cam.yaml
```

The node picks up `~/.ros/camera_info/default_cam.yaml` automatically via `usb_cam`.

---

#### Running `aruco_node` with webcam

```bash
# terminal 1 — webcam driver
ros2 run usb_cam usb_cam_node_exe --ros-args \
    -p video_device:=/dev/video0 \
    -p framerate:=30.0 \
    -p image_width:=640 \
    -p image_height:=480

# terminal 2 — aruco node
ros2 run perception aruco_node --ros-args \
    --remap /image:=/image_raw \
    -p camera_info_topic:=/camera_info \
    -p config_path:=$HOME/2026-Rover-Code/ros2_ws/src/perception/config/aruco_config.yaml

# terminal 3 — view debug image
ros2 run rqt_image_view rqt_image_view
# select /perception/aruco_debug

# terminal 4 — check TF tree (optional)
cd /tmp && ros2 run tf2_tools view_frames
# opens frames.pdf
```

#### Running `yolo_node` with webcam

```bash
# terminal 1 — webcam driver (same as above)

# terminal 2 — yolo node
ros2 run perception yolo_node --ros-args \
    --remap /image:=/image_raw \
    -p model_path:=$HOME/2026-Rover-Code/ros2_ws/src/perception/perception/models/hammer/best.onnx \
    -p confidence_threshold:=0.5

# terminal 3 — view debug image
ros2 run rqt_image_view rqt_image_view
# select /perception/yolo_debug
```

---

### With ZED 2i

> **Prerequisites:** CUDA, ZED SDK, `zed-ros2-wrapper` + `zed-ros2-interfaces` built in `/src`.
>
> ZED publishes factory calibration automatically — no calibration file needed.

#### Running `aruco_node` with ZED

```bash
# terminal 1 — ZED driver (fast config for dev)
ros2 launch zed_wrapper zed_camera.launch.py \
    camera_model:=zed2i \
    general.video_resolution:=VGA \
    general.pub_frame_rate:=10.0 \
    depth.depth_mode:=NONE

# terminal 2 — aruco node (camera_info_topic defaults to ZED, omit -p flag)
ros2 run perception aruco_node --ros-args \
    --remap /image:=/zed/zed_node/rgb/color/rect/image \
    -p config_path:=$HOME/2026-Rover-Code/ros2_ws/src/perception/config/aruco_config.yaml
```

#### Running `yolo_node` with ZED

```bash
ros2 run perception yolo_node --ros-args \
    --remap /image:=/zed/zed_node/rgb/color/rect/image \
    -p model_path:=$HOME/2026-Rover-Code/ros2_ws/src/perception/perception/models/hammer/best.onnx \
    -p confidence_threshold:=0.5
```

---

## TODOs

### `aruco_node`
- [ ] integrate ZED depth map to get 3D (X, Y, Z) tag coordinates
- [ ] implement mission controller nodes that subscribe to pose topics and publish drive/arm commands

### `yolo_node`
- [ ] test with webcam + real objects
- [ ] train + export mallet model
- [ ] train + export bottle model
- [ ] tune confidence threshold per-class as needed
- [ ] integrate with aruco_node for combined obstacle avoidance

### Deployment
- [ ] add `requirements.txt` install to `orin.Dockerfile` and `nano.Dockerfile`