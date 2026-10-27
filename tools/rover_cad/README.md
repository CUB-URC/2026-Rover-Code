# rover_cad: Fusion 360 → URDF → Gazebo

Turns the rover CAD into `ros2_ws/src/rover_description` (URDF + meshes) and
`ros2_ws/src/simulation/urdf/rover_gazebo.xacro` (Gazebo-only parts).

```
Fusion 360 ──ExportRoverLinks──▶ export dir ──build_urdf.py + rover_config.yaml──▶ rover_description/
 (CAD)          (Windows/Mac)    links.json + meshes/                               simulation/urdf/rover_gazebo.xacro
```

| File | What it is |
|---|---|
| `ExportRoverLinks/` | Fusion 360 script. Groups CAD bodies into URDF links and writes `links.json` + world-frame STL meshes |
| `rover_config.yaml` | How the links fit together: joints, pivots, wheels, the rocker differential, collision shapes and limits |
| `build_urdf.py` | Generator. Writes the URDF, re-framed meshes, `generated.yaml`, `preview.png`, `build_report.txt` and the Gazebo xacro |
| `make_terrain_world.py` | Generates `simulation/worlds/rover_course.sdf` and its terrain meshes |
| `placeholder_export/` | Stand-in export: boxes and cylinders with the real masses and pivot positions. It's what the current model is built from |

## Regenerate

From the repo root (needs `python3-numpy python3-yaml`, plus `python3-matplotlib` for the preview):

```bash
python3 tools/rover_cad/build_urdf.py tools/rover_cad/rover_config.yaml
python3 tools/rover_cad/make_terrain_world.py        # only if you changed the course
cd ros2_ws && colcon build --symlink-install --packages-up-to simulation
```

Then open `ros2_ws/src/rover_description/preview.png` and check that the gold
pivot dots sit on the real pivot holes and axles. Commit the regenerated files
together with the config change that produced them.

## Getting the real rover geometry in

1. In Fusion 360: Utilities → Add-Ins → Scripts and Add-Ins → (+) → pick
   `ExportRoverLinks/`. Set `OUTPUT_DIR` at the top of the script and run it in
   `MODE = "export"`. (`MODE = "dump"` writes the component tree, for filling in `LINKS`.)
2. Point `export_dir` in `rover_config.yaml` at that folder.
3. Regenerate (above). If the visual meshes are huge, `visual_max_triangles` /
   `collision_max_triangles` cap them. That needs `pip install fast-simplification`.

Raw CAD exports can be hundreds of MB, so don't commit the export folder itself.
Commit only the generated (simplified) meshes.
