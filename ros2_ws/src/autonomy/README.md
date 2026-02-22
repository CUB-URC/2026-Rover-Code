# Autonomy

The autonomy package leverages Nav2 for autonomous navigation using the ZED 2i stereo camera.

******


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

