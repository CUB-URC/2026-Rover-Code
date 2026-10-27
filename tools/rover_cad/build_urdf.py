#!/usr/bin/env python3
"""
build_urdf.py - turns the Fusion export (links.json + meshes/) and
rover_config.yaml into the rover_description and simulation ROS 2 packages.

    python3 tools/rover_cad/build_urdf.py tools/rover_cad/rover_config.yaml

Writes into ros2_ws/src/rover_description/ (no Gazebo dependencies):
    urdf/rover.urdf            the robot description (links, joints, inertia)
    meshes/<link>.stl          meshes re-framed into each link's frame (ROS axes)
    config/generated.yaml      spawn height, wheel radius/separation (used by launch)
    preview.png                side + top view of links and pivots, to sanity-check
    build_report.txt           masses, joint positions, warnings

Writes into ros2_ws/src/simulation/:
    urdf/rover_gazebo.xacro    Gazebo-only parts: friction, springs, plugins
"""
import json
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET
from xml.dom import minidom

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_SRC = os.path.join(os.path.dirname(os.path.dirname(HERE)), "ros2_ws", "src")
PKG = None  # set from config: package_dir
SIM_PKG = None  # set from config: sim_package_dir
PKG_NAME = "rover_description"
GZ_MODEL = "/model/rover"  # gz topic prefix, see bridge.yaml
LATERAL = np.array([0.0, 1.0, 0.0])

warnings = []


def warn(msg):
    warnings.append(msg)
    print("WARNING:", msg)


def die(msg):
    print("\nERROR:", msg)
    sys.exit(1)


# stl i/o
def read_stl(path):
    with open(path, "rb") as f:
        data = f.read()
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if 84 + n * 50 == len(data):
            rec = np.frombuffer(data[84:], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
            return rec["v"].reshape(-1, 3, 3).astype(np.float64)
    # ascii fallback
    verts = [list(map(float, ln.split()[1:4])) for ln in data.decode("ascii", "ignore").splitlines()
             if ln.strip().startswith("vertex")]
    return np.array(verts, dtype=np.float64).reshape(-1, 3, 3)


def write_stl(path, tris):
    tris = tris.astype(np.float32)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)
    rec = np.zeros(len(tris), dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    rec["n"] = n
    rec["v"] = tris.reshape(-1, 9)
    with open(path, "wb") as f:
        f.write(b"build_urdf".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        f.write(rec.tobytes())


# mesh simplification
def simplify(tris, max_tris, name):
    if not max_tris or len(tris) <= max_tris:
        return tris
    try:
        import fast_simplification
    except ImportError:
        warn("%s has %d triangles - install the simplifier for a faster sim:  "
             "pip install --break-system-packages fast-simplification" % (name, len(tris)))
        return tris
    pts = tris.reshape(-1, 3)
    key = np.round(pts / 1e-6).astype(np.int64)  # merge verts within 1 micron
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    verts = uniq.astype(np.float64) * 1e-6
    faces = inv.reshape(-1, 3)
    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])]
    if len(faces) <= max_tris:
        return verts[faces]
    v2, f2 = fast_simplification.simplify(verts.astype(np.float32), faces.astype(np.int64),
                                          target_reduction=1.0 - max_tris / len(faces))
    print("  simplified %-20s %8d -> %7d triangles" % (name, len(tris), len(f2)))
    return np.asarray(v2, dtype=np.float64)[np.asarray(f2)]


# axes
def axis_vec(s):
    s = s.strip().upper()
    sign = -1.0 if s.startswith("-") else 1.0
    letter = s.lstrip("+-")
    if letter not in "XYZ" or len(letter) != 1:
        die("fusion_axes entries must look like +X, -Y, +Z (got %r)" % s)
    v = np.zeros(3)
    v["XYZ".index(letter)] = sign
    return v


def fusion_to_ros_matrix(cfg):
    fa = cfg.get("fusion_axes", {}) or {}
    fwd = axis_vec(fa.get("forward", "+X"))
    up = axis_vec(fa.get("up", "+Y"))
    if abs(fwd @ up) > 0.5:
        die("fusion_axes.forward and .up must be different axes")
    left = np.cross(up, fwd)
    return np.vstack([fwd, left, up])  # rows: ROS x, y, z expressed in Fusion axes


# helpers
def fmt(v, nd=6):
    return " ".join(("%.*f" % (nd, x)).rstrip("0").rstrip(".") if abs(x) > 1e-12 else "0" for x in v)


def valid_inertia(I):
    w = np.linalg.eigvalsh((I + I.T) / 2)
    if np.any(w <= 0):
        return False
    a, b, c = sorted(w)
    return a + b >= c * 0.999


def box_inertia(m, size):
    x, y, z = size
    return np.diag([m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12])


# main
def main():
    if len(sys.argv) < 2:
        die("usage: python3 tools/build_urdf.py rover_config.yaml")
    cfg_path = sys.argv[1]
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    global PKG, SIM_PKG
    cfg_dir = os.path.dirname(os.path.abspath(cfg_path))

    def cfg_path_of(key, default):
        p = cfg.get(key)
        return os.path.join(cfg_dir, os.path.expanduser(p)) if p else default
    PKG = cfg_path_of("package_dir", os.path.join(REPO_SRC, "rover_description"))
    SIM_PKG = cfg_path_of("sim_package_dir", os.path.join(REPO_SRC, "simulation"))
    print("Writing packages into:", PKG, "and", SIM_PKG)
    export_dir = cfg_path_of("export_dir", None)
    if not export_dir:
        die("export_dir is not set in %s" % cfg_path)
    if not os.path.isdir(export_dir):
        die("export_dir not found: %s\n  (from WSL, Windows paths look like /mnt/c/Users/You/Desktop/rover_export)" % export_dir)
    with open(os.path.join(export_dir, "links.json")) as f:
        ex = json.load(f)
    for w in ex.get("warnings", []):
        warn("[from Fusion] " + w)

    R = fusion_to_ros_matrix(cfg)
    points = {k: R @ np.array(v) for k, v in (ex.get("points") or {}).items()}

    # load links into ROS frame
    links = {}
    overrides = cfg.get("mass_override_kg") or {}
    for name, d in ex["links"].items():
        stl = os.path.join(export_dir, "meshes", name + ".stl")
        if not os.path.exists(stl):
            die("missing mesh %s" % stl)
        tris = read_stl(stl) @ R.T
        flat = tris.reshape(-1, 3)
        lo, hi = flat.min(0), flat.max(0)
        m = float(d["mass"])
        com = R @ np.array(d["com"])
        I = R @ np.array(d["inertia"]) @ R.T
        if name in overrides:
            k = float(overrides[name]) / m if m > 0 else 0
            if m > 0:
                I = I * k
            m = float(overrides[name])
        if m <= 0:
            warn("%s has zero mass (no material in Fusion?) - using 1 kg box estimate" % name)
            m, com = 1.0, (lo + hi) / 2
            I = box_inertia(m, hi - lo)
        elif not valid_inertia(I):
            warn("%s inertia looked invalid (broken bodies?) - using bounding-box estimate" % name)
            I = box_inertia(m, hi - lo)
        I = I + np.eye(3) * 1e-6
        I[np.abs(I) < 1e-10] = 0.0
        links[name] = dict(tris=tris, lo=lo, hi=hi, mass=m, com=com, I=I)
        print("  loaded %-20s %7.2f kg  %7d tris" % (name, m, len(tris)))

    if "base_link" not in links:
        die("links.json has no base_link (the chassis). Check LINKS in the Fusion script.")

    centerline_y = (links["base_link"]["lo"][1] + links["base_link"]["hi"][1]) / 2
    joints_cfg = cfg["joints"]

    # resolve joint origins (world, ROS axes)
    bo = cfg.get("base_link_origin_mm")
    origin_w = {"base_link": (R @ (np.array(bo, float) / 1000.0)) if bo is not None
                else (links["base_link"]["lo"] + links["base_link"]["hi"]) / 2}
    joint_pos = {}

    def resolve(jname, stack=()):
        if jname in joint_pos:
            return joint_pos[jname]
        if jname in stack:
            die("mirror_of loop at %s" % jname)
        if jname not in joints_cfg:
            die("unknown joint %r" % jname)
        j = joints_cfg[jname]
        o = j.get("origin")
        if isinstance(o, str) and o.startswith("point:"):
            pname = o.split(":", 1)[1]
            if pname not in points:
                die("joint %s wants construction point %r, but the export only has: %s\n"
                    "  -> create it in Fusion and re-run the export, or use mm: [x,y,z]"
                    % (jname, pname, ", ".join(sorted(points)) or "(none)"))
            p = points[pname]
        elif o == "wheel_center":
            c = j["child"]
            if c not in links:
                die("joint %s: child link %s not in export" % (jname, c))
            p = (links[c]["lo"] + links[c]["hi"]) / 2
        elif o == "com":
            p = links[j["child"]]["com"]
        elif isinstance(o, dict) and "mm" in o:
            p = R @ (np.array(o["mm"], float) / 1000.0)
        elif isinstance(o, dict) and "mirror_of" in o:
            q = resolve(o["mirror_of"], stack + (jname,)).copy()
            q[1] = 2 * centerline_y - q[1]
            p = q
        else:
            die("joint %s: origin %r not understood" % (jname, o))
        joint_pos[jname] = np.array(p, float)
        return joint_pos[jname]

    children = {}
    for jname, j in joints_cfg.items():
        if j["child"] not in links:
            die("joint %s: child link %r is not in the Fusion export (exported: %s)"
                % (jname, j["child"], ", ".join(links)))
        if j["parent"] not in links:
            die("joint %s: parent link %r is not in the Fusion export" % (jname, j["parent"]))
        if j["child"] in children:
            die("link %s is the child of two joints" % j["child"])
        children[j["child"]] = jname
        origin_w[j["child"]] = resolve(jname)

    for ln in links:
        if ln != "base_link" and ln not in children:
            warn("link %s is exported but no joint uses it - it will be left out. "
                 "(Use __ignore__ in Fusion, or add a 'fixed' joint)" % ln)

    # wheels
    wcfg = cfg["wheels"]
    wheel_joints = list(wcfg["left"]) + list(wcfg["right"])
    wheel_links = {joints_cfg[j]["child"] for j in wheel_joints}
    radius, width = wcfg.get("radius_m"), wcfg.get("width_m")
    if radius is None or width is None:
        rs, ws = [], []
        for wl in wheel_links:
            ext = links[wl]["hi"] - links[wl]["lo"]
            rs.append(max(ext[0], ext[2]) / 2)
            ws.append(ext[1])
        radius = radius if radius is not None else float(np.median(rs))
        width = width if width is not None else float(np.median(ws))
        if max(rs) - min(rs) > 0.01:
            warn("measured wheel radii differ by %.1f mm - check wheel links contain only the wheel"
                 % ((max(rs) - min(rs)) * 1000))
    ly = np.mean([joint_pos[j][1] for j in wcfg["left"]])
    ry = np.mean([joint_pos[j][1] for j in wcfg["right"]])
    separation = float(abs(ly - ry))
    if ly < ry:
        warn("your 'left' wheels are on the RIGHT side in ROS axes - fusion_axes.forward is probably reversed")

    # URDF
    robot = ET.Element("robot", name=cfg.get("robot_name", "rover"))
    colors = {"wheel": (0.08, 0.08, 0.08), "base_link": (0.62, 0.22, 0.17), "other": (0.85, 0.83, 0.78)}
    for cname, rgb in colors.items():
        mat = ET.SubElement(robot, "material", name="mat_" + cname)
        ET.SubElement(mat, "color", rgba=fmt(list(rgb) + [1.0], 3))

    os.makedirs(os.path.join(PKG, "meshes"), exist_ok=True)
    os.makedirs(os.path.join(PKG, "urdf"), exist_ok=True)
    os.makedirs(os.path.join(PKG, "config"), exist_ok=True)

    used = ["base_link"] + [j["child"] for j in joints_cfg.values()]
    coll_cfg = cfg.get("collision") or {}

    def collision_mesh(ln, tris_local):
        cap = cfg.get("collision_max_triangles", 15000)
        write_stl(os.path.join(PKG, "meshes", ln + "_collision.stl"), simplify(tris_local, cap, ln + " (collision)"))
        return "package://%s/meshes/%s_collision.stl" % (PKG_NAME, ln)
    for ln in used:
        L = links[ln]
        o = origin_w[ln]
        write_stl(os.path.join(PKG, "meshes", ln + ".stl"),
                  simplify(L["tris"] - o, cfg.get("visual_max_triangles", 30000), ln))
        link = ET.SubElement(robot, "link", name=ln)
        inr = ET.SubElement(link, "inertial")
        ET.SubElement(inr, "origin", xyz=fmt(L["com"] - o), rpy="0 0 0")
        ET.SubElement(inr, "mass", value="%.4f" % L["mass"])
        I = L["I"]
        ET.SubElement(inr, "inertia", ixx="%.6g" % I[0, 0], ixy="%.6g" % I[0, 1], ixz="%.6g" % I[0, 2],
                      iyy="%.6g" % I[1, 1], iyz="%.6g" % I[1, 2], izz="%.6g" % I[2, 2])
        vis = ET.SubElement(link, "visual")
        ET.SubElement(vis, "origin", xyz="0 0 0", rpy="0 0 0")
        g = ET.SubElement(vis, "geometry")
        ET.SubElement(g, "mesh", filename="package://%s/meshes/%s.stl" % (PKG_NAME, ln))
        key = "wheel" if ln in wheel_links else ("base_link" if ln == "base_link" else "other")
        ET.SubElement(vis, "material", name="mat_" + key)

        wmode = wcfg.get("collision", "lugs" if wcfg.get("lugs") else "cylinder")
        if ln in wheel_links and wmode == "mesh":
            col = ET.SubElement(link, "collision")
            ET.SubElement(col, "origin", xyz="0 0 0", rpy="0 0 0")
            g = ET.SubElement(col, "geometry")
            ET.SubElement(g, "mesh", filename=collision_mesh(ln, L["tris"] - o))
        elif ln in wheel_links:
            lugs = (wcfg.get("lugs") or {}) if wmode == "lugs" else {}
            lug_h = float(lugs.get("height_m", 0)) if lugs else 0.0
            col = ET.SubElement(link, "collision")
            ET.SubElement(col, "origin", xyz="0 0 0", rpy="%.6f 0 0" % (math.pi / 2))
            g = ET.SubElement(col, "geometry")
            ET.SubElement(g, "cylinder", radius="%.4f" % (radius - lug_h), length="%.4f" % width)
            if lug_h > 0:
                n = int(lugs.get("count", 18))
                t = lugs.get("thickness_m", "auto")
                t = math.pi * radius / n if t in (None, "auto") else float(t)
                rm = radius - lug_h / 2
                for k in range(n):
                    th = 2 * math.pi * k / n
                    col = ET.SubElement(link, "collision", name="%s_lug_%d" % (ln, k))
                    ET.SubElement(col, "origin", xyz=fmt([rm * math.cos(th), 0, rm * math.sin(th)]),
                                  rpy="0 %.6f 0" % (-th))
                    g = ET.SubElement(col, "geometry")
                    ET.SubElement(g, "box", size=fmt([lug_h, width, t]))
        else:
            mode = coll_cfg.get(ln, "box")
            if mode == "box":
                col = ET.SubElement(link, "collision")
                ET.SubElement(col, "origin", xyz=fmt((L["lo"] + L["hi"]) / 2 - o), rpy="0 0 0")
                g = ET.SubElement(col, "geometry")
                ET.SubElement(g, "box", size=fmt(L["hi"] - L["lo"]))
            elif mode == "mesh":
                col = ET.SubElement(link, "collision")
                ET.SubElement(col, "origin", xyz="0 0 0", rpy="0 0 0")
                g = ET.SubElement(col, "geometry")
                ET.SubElement(g, "mesh", filename=collision_mesh(ln, L["tris"] - o))

    diff = cfg.get("differential") or {}
    dmode = diff.get("mode", "spring")
    rocker_joints = [diff.get("left_joint", "rocker_left_joint"), diff.get("right_joint", "rocker_right_joint")]
    for rj in rocker_joints:
        if rj not in joints_cfg:
            die("differential: joint %r not found in joints (set differential.left_joint/right_joint)" % rj)
    if dmode not in ("spring", "linkage", "mimic", "none"):
        die("differential.mode must be spring, linkage, mimic or none (got %r)" % dmode)
    ratio = float(diff.get("ratio", 1.0))
    comp = wcfg.get("compliance") or {}
    tire_joints = []
    for jname, j in joints_cfg.items():
        parent = j["parent"]
        xyz = origin_w[j["child"]] - origin_w[parent]
        if comp and j["child"] in wheel_links:
            # tire compliance: prismatic spring between parent and wheel
            mount = j["child"] + "_tire"
            ml = ET.SubElement(robot, "link", name=mount)
            mi = ET.SubElement(ml, "inertial")
            ET.SubElement(mi, "origin", xyz="0 0 0", rpy="0 0 0")
            ET.SubElement(mi, "mass", value="0.1")
            ET.SubElement(mi, "inertia", ixx="1e-4", ixy="0", ixz="0", iyy="1e-4", iyz="0", izz="1e-4")
            tj = ET.SubElement(robot, "joint", name=mount + "_joint", type="prismatic")
            ET.SubElement(tj, "parent", link=parent)
            ET.SubElement(tj, "child", link=mount)
            ET.SubElement(tj, "origin", xyz=fmt(xyz), rpy="0 0 0")
            ET.SubElement(tj, "axis", xyz="0 0 1")
            tr = float(comp.get("travel_m", 0.025))
            ET.SubElement(tj, "limit", lower="%g" % -tr, upper="%g" % tr, effort="5000", velocity="5")
            ET.SubElement(tj, "dynamics", damping="%g" % float(comp.get("damping_ns_per_m", 100)), friction="0")
            tire_joints.append(mount + "_joint")
            parent, xyz = mount, np.zeros(3)
        je = ET.SubElement(robot, "joint", name=jname, type=j["type"])
        ET.SubElement(je, "parent", link=parent)
        ET.SubElement(je, "child", link=j["child"])
        ET.SubElement(je, "origin", xyz=fmt(xyz), rpy="0 0 0")
        ax = LATERAL if j.get("axis", "lateral") == "lateral" else np.array(j["axis"], float)
        ET.SubElement(je, "axis", xyz=fmt(ax / np.linalg.norm(ax)))
        if j["type"] == "revolute":
            lo, hi = j.get("limit_deg", [-45, 45])
            ET.SubElement(je, "limit", lower="%.4f" % math.radians(lo), upper="%.4f" % math.radians(hi),
                          effort="1000", velocity="10")
            damp = diff.get("damping_nms_per_rad", 5) if jname in rocker_joints else 1.0
            ET.SubElement(je, "dynamics", damping="%.3f" % damp, friction="0")
        elif j["type"] == "continuous":
            ET.SubElement(je, "limit", effort="200", velocity="30")
            ET.SubElement(je, "dynamics", damping="0.1", friction="0")
        if dmode == "mimic" and len(rocker_joints) == 2 and jname == rocker_joints[1]:
            ET.SubElement(je, "mimic", joint=rocker_joints[0], multiplier="%g" % -ratio, offset="0")

    # gazebo extensions, written to simulation/urdf/rover_gazebo.xacro
    gzroot = ET.Element("robot", {"xmlns:xacro": "http://www.ros.org/wiki/xacro"})
    for wl in sorted(wheel_links):
        gz = ET.SubElement(gzroot, "gazebo", reference=wl)
        mu = str(wcfg.get("friction", 1.0))
        ET.SubElement(gz, "mu1").text = mu
        ET.SubElement(gz, "mu2").text = mu
        ET.SubElement(gz, "kp").text = "1000000"
        ET.SubElement(gz, "kd").text = "100"
        ET.SubElement(gz, "minDepth").text = "0.001"
    for tj in tire_joints:
        gz = ET.SubElement(gzroot, "gazebo", reference=tj)
        ET.SubElement(gz, "springStiffness").text = "%g" % float(comp.get("stiffness_n_per_m", 5000))
        ET.SubElement(gz, "springReference").text = "0"
    spring_k = {"spring": diff.get("spring_stiffness_nm_per_rad", 150),
                "linkage": diff.get("linkage_backup_spring_nm_per_rad", 30)}.get(dmode, 0)
    if spring_k:
        for rj in rocker_joints:
            gz = ET.SubElement(gzroot, "gazebo", reference=rj)
            ET.SubElement(gz, "springStiffness").text = str(spring_k)
            ET.SubElement(gz, "springReference").text = "0"

    drv = cfg.get("drive") or {}
    gz = ET.SubElement(gzroot, "gazebo")
    if dmode == "linkage":
        # see simulation/src/RockerDifferential.cc
        pf = ET.SubElement(gz, "plugin", filename="RockerDifferential", name="rover::RockerDifferential")
        for tag, val in [("left_joint", rocker_joints[0]), ("right_joint", rocker_joints[1]),
                         ("ratio", "%g" % ratio),
                         ("stiffness", "%g" % float(diff.get("linkage_stiffness_nm_per_rad", 3000))),
                         ("damping", "%g" % float(diff.get("linkage_damping_nms_per_rad", 60))),
                         ("max_torque", "%g" % float(diff.get("linkage_max_torque_nm", 600)))]:
            ET.SubElement(pf, tag).text = val
    p = ET.SubElement(gz, "plugin", filename="gz-sim-diff-drive-system", name="gz::sim::systems::DiffDrive")
    for jn in wcfg["left"]:
        ET.SubElement(p, "left_joint").text = jn
    for jn in wcfg["right"]:
        ET.SubElement(p, "right_joint").text = jn
    for tag, val in [("wheel_separation", "%.4f" % separation), ("wheel_radius", "%.4f" % radius),
                     ("odom_publish_frequency", "30"), ("topic", GZ_MODEL + "/cmd_vel"), ("odom_topic", GZ_MODEL + "/odometry"),
                     ("tf_topic", GZ_MODEL + "/tf"), ("frame_id", "odom"), ("child_frame_id", "base_link"),
                     ("max_linear_velocity", str(drv.get("max_linear_mps", 1.0))),
                     ("min_linear_velocity", str(-drv.get("max_linear_mps", 1.0))),
                     ("max_angular_velocity", str(drv.get("max_angular_rps", 1.5))),
                     ("min_angular_velocity", str(-drv.get("max_angular_rps", 1.5))),
                     ("max_linear_acceleration", str(drv.get("max_accel_mps2", 1.5))),
                     ("min_linear_acceleration", str(-drv.get("max_accel_mps2", 1.5)))]:
        ET.SubElement(p, tag).text = val
    p2 = ET.SubElement(gz, "plugin", filename="gz-sim-joint-state-publisher-system",
                       name="gz::sim::systems::JointStatePublisher")
    ET.SubElement(p2, "topic").text = GZ_MODEL + "/joint_states"

    src = os.path.basename(export_dir.rstrip("/"))
    note = ("<!-- GENERATED by tools/rover_cad/build_urdf.py from %s. "
            "Edit tools/rover_cad/rover_config.yaml and re-run instead of editing this file. -->\n" % src)
    urdf_path = os.path.join(PKG, "urdf", "rover.urdf")
    gz_path = os.path.join(SIM_PKG, "urdf", "rover_gazebo.xacro")
    os.makedirs(os.path.dirname(gz_path), exist_ok=True)
    for root, path in ((robot, urdf_path), (gzroot, gz_path)):
        xml = minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")
        with open(path, "w") as f:
            f.write("<?xml version=\"1.0\"?>\n" + note + xml.split("\n", 1)[1])

    # spawn height: lowest wheel bottom below base_link origin
    base_o = origin_w["base_link"]
    lowest = min(origin_w[w][2] - radius for w in wheel_links) - base_o[2]
    for ln in used:
        if ln not in wheel_links and coll_cfg.get(ln, "box") != "none":
            lowest = min(lowest, links[ln]["lo"][2] - base_o[2])
    spawn_z = -lowest + 0.03
    total_mass = sum(links[ln]["mass"] for ln in used)
    gen = {"spawn_z": round(float(spawn_z), 4), "wheel_radius": round(radius, 4),
           "wheel_separation": round(separation, 4), "wheel_width": round(width, 4),
           "total_mass_kg": round(total_mass, 3),
           "differential": {"mode": dmode, "left_joint": rocker_joints[0], "right_joint": rocker_joints[1],
                            "ratio": ratio,
                            "stiffness": float(diff.get("linkage_stiffness_nm_per_rad", 3000)),
                            "damping": float(diff.get("linkage_damping_nms_per_rad", 60)),
                            "max_torque": float(diff.get("linkage_max_torque_nm", 600))}}
    with open(os.path.join(PKG, "config", "generated.yaml"), "w") as f:
        yaml.safe_dump(gen, f, sort_keys=False)

    # report
    lines = ["BUILD REPORT for %s" % cfg.get("robot_name"), "",
             "Total mass: %.2f kg" % total_mass,
             "Wheel radius %.1f mm, width %.1f mm, track (separation) %.1f mm"
             % (radius * 1000, width * 1000, separation * 1000),
             "Spawn height: %.3f m" % spawn_z, "", "Links (ROS axes, world, mm):"]
    for ln in used:
        lines.append("  %-20s mass %7.2f kg   origin %s" % (ln, links[ln]["mass"],
                                                            fmt(origin_w[ln] * 1000, 1)))
    lines += ["", "Warnings:"] + ["  - " + w for w in warnings] if warnings else ["", "Warnings: none"]
    with open(os.path.join(PKG, "build_report.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")

    preview(links, used, origin_w, joints_cfg, wheel_links, radius)

    print("\n" + "\n".join(lines))
    print("\nWrote %s\n      %s" % (urdf_path, gz_path))
    print("Next:  cd ros2_ws && colcon build --symlink-install && source install/setup.bash")


def preview(links, used, origin_w, joints_cfg, wheel_links, radius):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Rectangle
    except ImportError:
        print("(matplotlib not installed - skipping preview.png)")
        return
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    for ax, (a, b), title in [(axes[0], (0, 2), "SIDE view (X fwd ->, Z up)"),
                              (axes[1], (0, 1), "TOP view (X fwd ->, Y left)")]:
        for ln in used:
            L = links[ln]
            t = L["tris"]
            idx = np.linspace(0, len(t) - 1, min(len(t), 6000)).astype(int)
            pts = t[idx].reshape(-1, 3)
            c = "0.1" if ln in wheel_links else ("tab:red" if ln == "base_link" else "tab:blue")
            ax.scatter(pts[:, a], pts[:, b], s=0.2, c=c, alpha=0.25)
        for jn, j in joints_cfg.items():
            p = origin_w[j["child"]]
            ax.plot(p[a], p[b], "o", ms=7, mfc="gold", mec="k", zorder=5)
            ax.annotate(jn.replace("_joint", ""), (p[a], p[b]), fontsize=7,
                        xytext=(4, 4), textcoords="offset points")
            if j["child"] in wheel_links and b == 2:
                ax.add_patch(Circle((p[a], p[b]), radius, fill=False, ec="green", lw=1))
        ax.set_title(title)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("z [m]")
    axes[1].set_xlabel("x [m]")
    axes[1].set_ylabel("y [m]")
    fig.suptitle("Gold dots = joint pivots. They should sit exactly on pivot holes / axle centers. "
                 "Green = wheel collision cylinder.", fontsize=10)
    fig.tight_layout()
    out = os.path.join(PKG, "preview.png")
    fig.savefig(out, dpi=110)
    print("Wrote %s  <- open this and check the pivots!" % out)


if __name__ == "__main__":
    main()
