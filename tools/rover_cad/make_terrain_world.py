#!/usr/bin/env python3
"""
make_terrain_world.py - generates the rover test course for Gazebo.

    python3 tools/rover_cad/make_terrain_world.py [path/to/ros2_ws/src/simulation]

Writes:
    worlds/rover_course.sdf       the world (load with world:=rover_course)
    worlds/terrain/*.stl          generated terrain meshes

Edit the ZONES section below (sizes, angles, heights) and re-run, then
colcon build. Rover spawns at the origin facing +X.

COURSE MAP (top view, meters; rover starts at 0,0 facing ->)

        +Y
         |   [stairs 10cm]   [stairs 18cm]   [side slope 15deg]
         |     x 3..8          x 10..15         x 17..24
   y 4..7|
         |
  -X <---+--- START ---> [wavy ground] [rough terrain] [boulder field]  +X
  behind:|                 x 3..9        x 11..17         x 19..25
  curbs, |
  logs,  |   [ramp 10deg]    [ramp 20deg]    [ramp 30deg]
  steps  |     x 3..9          x 11..17         x 19..25
         |  y -4..-7
        -Y
"""
import math
import os
import struct
import sys

import numpy as np

PKG = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "ros2_ws", "src", "simulation")
WORLDS = os.path.join(PKG, "worlds")
TERRAIN = os.path.join(WORLDS, "terrain")
URI = "model://simulation/worlds/terrain/"
RES = 0.05            # terrain mesh resolution (m)
rng = np.random.default_rng(7)


# ------------------------------------------------------------ mesh helpers
def write_stl(path, tris):
    tris = np.asarray(tris, dtype=np.float32)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)
    rec = np.zeros(len(tris), dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    rec["n"] = n
    rec["v"] = tris.reshape(-1, 9)
    with open(path, "wb") as f:
        f.write(b"rover course".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        f.write(rec.tobytes())


def heightfield_solid(fn, x0, x1, y0, y1, base=-0.3, res=RES):
    """Closed solid whose top surface is z = fn(X, Y) over a rectangle.
    Coordinates are local (the model pose places it)."""
    xs = np.linspace(x0, x1, max(2, int(round((x1 - x0) / res)) + 1))
    ys = np.linspace(y0, y1, max(2, int(round((y1 - y0) / res)) + 1))
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    Z = np.maximum(fn(X, Y), 0.0)
    P = np.stack([X, Y, Z], -1)
    B = P.copy()
    B[..., 2] = base
    tris = []
    a, b, c, d = P[:-1, :-1], P[1:, :-1], P[1:, 1:], P[:-1, 1:]
    tris.append(np.stack([a, b, c], -2).reshape(-1, 3, 3))      # top, normal +z
    tris.append(np.stack([a, c, d], -2).reshape(-1, 3, 3))
    a, b, c, d = B[:-1, :-1], B[1:, :-1], B[1:, 1:], B[:-1, 1:]
    tris.append(np.stack([a, c, b], -2).reshape(-1, 3, 3))      # bottom, normal -z
    tris.append(np.stack([a, d, c], -2).reshape(-1, 3, 3))

    def wall(top, bot, flip):
        t1 = np.stack([top[:-1], bot[:-1], bot[1:]], 1)
        t2 = np.stack([top[:-1], bot[1:], top[1:]], 1)
        w = np.concatenate([t1, t2])
        return w[:, ::-1] if flip else w

    tris.append(wall(P[:, 0], B[:, 0], False))    # y = y0 side, normal -y
    tris.append(wall(P[:, -1], B[:, -1], True))   # y = y1 side, normal +y
    tris.append(wall(P[0, :], B[0, :], True))     # x = x0 side, normal -x
    tris.append(wall(P[-1, :], B[-1, :], False))  # x = x1 side, normal +x
    return np.concatenate(tris)


def taper(X, x0, x1, ramp=1.0):
    """0 at the patch edges, rising smoothly to 1 over `ramp` meters."""
    t = np.clip(np.minimum(X - x0, x1 - X) / ramp, 0, 1)
    return 0.5 - 0.5 * np.cos(np.pi * t)


def smooth_noise(X, Y, scale, octaves=3, seed=0):
    r = np.random.default_rng(seed)
    out = np.zeros_like(X)
    amp, freq = 1.0, 1.0 / scale
    for _ in range(octaves):
        for _ in range(6):
            kx, ky = r.normal(0, freq, 2)
            ph = r.uniform(0, 2 * np.pi)
            out += amp * np.sin(kx * 2 * np.pi * X + ky * 2 * np.pi * Y + ph)
        amp *= 0.5
        freq *= 2.0
    return out / np.abs(out).max()


# ------------------------------------------------------------ SDF helpers
models = []
COLORS = {"dirt": "0.66 0.52 0.38 1", "rock": "0.42 0.38 0.35 1", "concrete": "0.62 0.62 0.6 1",
          "wood": "0.45 0.32 0.2 1", "ramp": "0.55 0.47 0.4 1", "red": "0.7 0.25 0.2 1"}


def surface(mu=1.0):
    return "<surface><friction><ode><mu>%g</mu><mu2>%g</mu2></ode></friction></surface>" % (mu, mu)


def material(c):
    return "<material><ambient>%s</ambient><diffuse>%s</diffuse></material>" % (COLORS[c], COLORS[c])


def add_mesh(name, tris, pose="0 0 0 0 0 0", color="dirt"):
    write_stl(os.path.join(TERRAIN, name + ".stl"), tris)
    g = "<geometry><mesh><uri>%s%s.stl</uri></mesh></geometry>" % (URI, name)
    models.append(f"""    <model name="{name}"><static>true</static><pose>{pose}</pose>
      <link name="link">
        <collision name="c">{g}{surface()}</collision>
        <visual name="v">{g}{material(color)}</visual>
      </link></model>""")


def add_shape(name, geom, pose, color="concrete", mu=1.0):
    models.append(f"""    <model name="{name}"><static>true</static><pose>{pose}</pose>
      <link name="link">
        <collision name="c"><geometry>{geom}</geometry>{surface(mu)}</collision>
        <visual name="v"><geometry>{geom}</geometry>{material(color)}</visual>
      </link></model>""")


def box(sx, sy, sz):
    return "<box><size>%g %g %g</size></box>" % (sx, sy, sz)


# ------------------------------------------------------------ ZONES
def build():
    os.makedirs(TERRAIN, exist_ok=True)

    # --- straight ahead: wavy ground (egg-crate waves, 12 cm amplitude)
    add_mesh("wavy_ground", heightfield_solid(
        lambda X, Y: (0.12 * (1 + np.sin(2 * np.pi * X / 1.4) * np.sin(2 * np.pi * Y / 1.8)))
        * taper(X, 3, 9, 1.0) * taper(Y, -2, 2, 0.6), 3, 9, -2, 2))

    # --- rough terrain: multi-scale noise, up to ~30 cm, plus a crater
    def rough(X, Y):
        h = 0.15 + 0.15 * smooth_noise(X, Y, 1.5, 3, seed=11)
        crater = -0.22 * np.exp(-((X - 14.5) ** 2 + (Y - 0.6) ** 2) / 0.35)
        return (h + crater) * taper(X, 11, 17, 1.2) * taper(Y, -2, 2, 0.8)
    add_mesh("rough_terrain", heightfield_solid(rough, 11, 17, -2, 2), color="dirt")

    # --- boulder field: half-buried spheres 10-35 cm tall + angular rocks
    for i in range(26):
        r = rng.uniform(0.12, 0.40)
        x, y = rng.uniform(19.5, 25), rng.uniform(-1.8, 1.8)
        add_shape("boulder_%d" % i, "<sphere><radius>%.3f</radius></sphere>" % r,
                  "%.2f %.2f %.3f 0 0 0" % (x, y, -r + rng.uniform(0.10, min(0.35, r))), "rock")
    for i in range(10):
        s = rng.uniform(0.15, 0.3)
        add_shape("rock_%d" % i, box(s * 1.4, s, s),
                  "%.2f %.2f %.3f %.2f %.2f %.2f" % (rng.uniform(19.5, 25), rng.uniform(-1.8, 1.8), s * 0.3,
                                                     rng.uniform(-.5, .5), rng.uniform(-.5, .5), rng.uniform(0, 3)), "rock")

    # --- ramps (right side): up-slope, 1.5 m flat top, down-slope; 50 cm tall
    for k, (deg, xs) in enumerate([(10, 3.0), (20, 11.0), (30, 19.0)]):
        h, top = 0.5, 1.5
        run = h / math.tan(math.radians(deg))
        x0, x1 = 0.0, 2 * run + top

        def ramp(X, Y, run=run, top=top, h=h):
            return h * np.clip(np.minimum(X / run, (2 * run + top - X) / run), 0, 1)
        add_mesh("ramp_%ddeg" % deg, heightfield_solid(ramp, x0, x1, 0, 3, res=0.04),
                 "%.2f -7 0 0 0 0" % xs, "ramp")

    # --- stairs (left side): up 4 steps, platform, down 4 steps; 3 m wide
    for rise, x0, label in [(0.10, 3.0, "10cm"), (0.18, 10.0, "18cm")]:
        tread, n = 0.55, 4
        for i in range(n):  # each step is a box from the ground up
            hgt = rise * (i + 1)
            add_shape("stairs_%s_up_%d" % (label, i), box(tread, 3, hgt),
                      "%.3f 5.5 %.3f 0 0 0" % (x0 + tread * (i + 0.5), hgt / 2), "concrete")
        plat_x = x0 + tread * n
        add_shape("stairs_%s_top" % label, box(1.2, 3, rise * n),
                  "%.3f 5.5 %.3f 0 0 0" % (plat_x + 0.6, rise * n / 2), "concrete")
        for i in range(n):
            hgt = rise * (n - i)
            add_shape("stairs_%s_down_%d" % (label, i), box(tread, 3, hgt),
                      "%.3f 5.5 %.3f 0 0 0" % (plat_x + 1.2 + tread * (i + 0.5), hgt / 2), "concrete")

    # --- side slope (left side, far): drive along X across a 15 deg cross-slope
    s = math.tan(math.radians(15))
    add_mesh("side_slope_15deg", heightfield_solid(
        lambda X, Y: s * Y * taper(X, 0, 7, 1.5), 0, 7, 0, 3.2), "17 4 0 0 0 0", "dirt")

    # --- behind the start: one-sided curbs (differential test), logs, single steps
    for i, h in enumerate([0.08, 0.15, 0.22]):
        add_shape("curb_left_%dcm" % int(h * 100), box(0.5, 0.8, h),
                  "%.2f 0.55 %.3f 0 0 0" % (-3 - 2.5 * i, h / 2), "red")
    for i, d in enumerate([0.10, 0.18, 0.26]):
        add_shape("log_%dcm" % int(d * 100), "<cylinder><radius>%g</radius><length>3</length></cylinder>" % (d / 2),
                  "%.2f -4.5 %.3f 1.5708 0 0" % (-3 - 2.5 * i, d / 2), "wood")

    world = f"""<?xml version="1.0"?>
<!-- GENERATED by tools/rover_cad/make_terrain_world.py - edit the script and re-run. -->
<sdf version="1.9">
  <world name="rover_course">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <scene><ambient>0.55 0.55 0.55 1</ambient><background>0.72 0.8 0.9 1</background><shadows>false</shadows></scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.95 0.92 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.3 -0.8</direction>
    </light>
    <model name="ground"><static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>300 300</size></plane></geometry>{surface()}</collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>300 300</size></plane></geometry>{material("dirt")}</visual>
      </link></model>
{chr(10).join(models)}
  </world>
</sdf>
"""
    with open(os.path.join(WORLDS, "rover_course.sdf"), "w") as f:
        f.write(world)
    n = sum(1 for _ in os.scandir(TERRAIN))
    print("Wrote %s (%d models, %d terrain meshes)" % (os.path.join(WORLDS, "rover_course.sdf"), len(models), n))


if __name__ == "__main__":
    build()
