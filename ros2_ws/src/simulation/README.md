# simulation

Gazebo **Harmonic** simulation of the rover on ROS 2 Humble. It speaks the same
topics as the real rover, so drive and autonomy code run against it unchanged.

| What | Where |
|---|---|
| Launch | `launch/sim.launch.py` |
| Worlds | `worlds/rover_course.sdf` (generated), `test_terrain.sdf`, `empty.sdf` |
| ROS ↔ Gazebo topics | `config/bridge.yaml` |
| Rover as Gazebo sees it | `urdf/rover_sim.urdf.xacro` = `rover_description/urdf/rover.urdf` + `urdf/rover_gazebo.xacro` (generated) |
| Rocker differential plugin | `src/RockerDifferential.cc` |

Why Harmonic: Humble's default pairing, Gazebo Fortress, reached end of life in
September 2026. Harmonic is supported until 2028 and is the version ROS 2 Jazzy
uses, so moving to Jazzy later doesn't change any of this. On Humble it comes
from the OSRF apt repo as `ros-humble-ros-gzharmonic`. **Don't** install
`ros-humble-ros-gz*` (the Fortress builds) alongside it: they conflict, and
`rosdep install` will try to pull them in for this package, so skip it here.

## Setup

**Docker (any Linux host):**

```bash
xhost +local:docker                       # let the container open windows
cd docker && docker compose run sim       # or: sim-nvidia, with nvidia-container-toolkit
# inside:
cd ros2_ws && colcon build --symlink-install --packages-up-to simulation
source install/setup.bash
```

**Native Ubuntu 22.04:** `scripts/setup_sim.sh` (installs Harmonic, removes
the conflicting Fortress packages, builds).

## Run

```bash
ros2 launch simulation sim.launch.py                      # rover_course, Gazebo window
ros2 launch simulation sim.launch.py world:=test_terrain rviz:=true
ros2 launch simulation sim.launch.py headless:=true       # no window (CI, slow machines)
```

Drive it like the real rover, via `/drive/cmd_vel`:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/drive/cmd_vel
ros2 run drive drive_input_node     # Xbox controller (drive branch)
```

## Topics

| ROS topic | Dir | Notes |
|---|---|---|
| `/drive/cmd_vel` | → sim | `geometry_msgs/Twist`, same as the real `drive_controller` |
| `/sim/odom` | sim → | DiffDrive wheel odometry. Perfect (no slip or noise), so it's under `/sim/`, not a real sensor name |
| `/joint_states` | sim → | rocker, bogie and wheel joints |
| `/tf` (`odom→base_link`) | sim → | only with `publish_odom_tf:=true` (default). Set it to `false` when autonomy's EKF publishes that transform |
| `/clock` | sim → | everything in the sim graph should use `use_sim_time:=true` |

Not simulated yet: the ZED (RGB, depth, point cloud, IMU, VIO odom on
`/zed/zed_node/*`) and GPS. That's the next step, so nav2 can run against the sim.

## Rover model

The rover's meshes are **placeholders** (boxes and cylinders) with the real
masses, wheel size and pivot positions. To get the real geometry, run the
Fusion export and regenerate. See `tools/rover_cad/README.md`.

Collision shapes: wheels are cylinders, and the other links use their (capped)
meshes. Mesh-on-mesh contact is the main thing that makes this sim slow, so keep
`collision_max_triangles` modest in `rover_config.yaml`.

## Troubleshooting

- **Low real-time factor** (bottom right of the Gazebo window): on laptops with
  two GPUs, Gazebo may be rendering on the integrated one. Use `sim-nvidia`, or
  natively: `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 launch ...`
- **"RockerDifferential plugin was not built"**: the workspace was built
  without Gazebo Harmonic's dev packages. Install them, then rebuild `simulation`.
  Without the plugin, the chassis only stays level through a weak backup spring.
- **Rover meshes don't show up / are invisible**: the workspace wasn't sourced
  in that terminal, or an older `rover_description` overlay (e.g. `~/rover_ws`)
  is sourced after this one.
