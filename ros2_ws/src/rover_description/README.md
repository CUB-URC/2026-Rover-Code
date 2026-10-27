# rover_description

URDF and meshes for the rover: links, joints, masses and inertia. It has no
Gazebo dependency, so RViz, the real rover, IK and nav can all use it. The
Gazebo-only parts (friction, springs, plugins) live in the `simulation` package.

Everything in `urdf/`, `meshes/`, `config/generated.yaml`, `preview.png` and
`build_report.txt` is **generated** by `tools/rover_cad/build_urdf.py`. Don't
edit them by hand. Edit `tools/rover_cad/rover_config.yaml` and re-run (see
`tools/rover_cad/README.md`).

> The meshes are currently **placeholders** (boxes and cylinders) carrying the
> real rover's masses and pivot positions. Replace with a real Fusion export

View the model and move each joint with sliders:

```bash
ros2 launch rover_description display.launch.py
```
