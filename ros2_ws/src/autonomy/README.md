# Autonomy

The autonomy package leverages Nav2 for autonomous navigation using the ZED 2i stereo camera.

---

> still needs a lot of work!

## Config Notes

### `nav2_params.yaml`

#### `collision_monitor`

ZED doesn't publish LaserScan natively => need to pipe ZED depth image through depthimage_to_laserscan:

```bash
   ros2 run depthimage_to_laserscan depthimage_to_laserscan_node \
     --ros-args \
     -r depth:=/zed/zed_node/depth/depth_registered \
     -r camera_info:=/zed/zed_node/depth/camera_info \
     -p output_frame:=base_link
```

## Testing

> still needs to be tested, commands + code might need tweaking

### NO ZED + NO GPS

```bash
# 1. build
cd ~/2026-Rover-Code/ros2_ws
colcon build --packages-select autonomy
source install/setup.bash

# 2. launch nav2 (static map→odom, no GPS)
ros2 launch autonomy nav2_bringup.py use_gps:=false use_rviz:=true

# 3. in a second terminal — run the demo controller (drives 2m forward in sim)
ros2 run autonomy autonomous_nav_controller.py --ros-args -p mode:=demo
```

### WITH ZED, NO GPS

```bash
# 1. build
cd ~/2026-Rover-Code/ros2_ws
colcon build --packages-select autonomy
source install/setup.bash

# 2. ZED Wrapper Launch
ros2 launch zed_wrapper zed.launch.py \
  --ros-args \
  -p general.camera_model:=zed2i \
  -p general.publish_tf:=true \
  -p general.self_calibration:=true \
  -p general.depth_mode:=PERFORMANCE \
  -p general.confidence_threshold:=50

# 3. in a second terminal — run the demo controller (drives 2m forward in sim)
ros2 launch autonomy nav2_bringup.py use_gps:=false use_rviz:=true
```

### SEARCH PATTERN DEMO

```bash
ros2 run autonomy autonomous_nav_controller.py --ros-args \
  -p mode:=search \
  -p goal_lat:=0.0 \
  -p goal_lon:=0.0 \
  -p search_radius:=5.0 \
  -p search_spacing:=1.5
```
