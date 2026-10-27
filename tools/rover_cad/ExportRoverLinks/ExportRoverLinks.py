"""
ExportRoverLinks - Fusion 360 script for URC rover -> URDF pipeline.

Doesn't need Fusion joints and doesn't care how deep components are nested.

TWO MODES (set MODE below):
  "dump"   -> writes rover_tree.txt: every component in your assembly with
              its full path. Use this to fill in LINKS.
  "export" -> groups every visible body into one of your URDF links, then
              writes for each link:
                 meshes/<link>.stl    (binary STL, world frame, METERS)
                 links.json           (mass, center of mass, inertia per link,
                                       plus pivot marker positions)

PIVOT MARKERS: for each suspension pivot, create a construction point at the
center of the pivot hole/shaft (Construct -> Point at Center of
Circle/Sphere/Torus) and rename it starting with "joint_", e.g.
joint_rocker_left, joint_bogie_left. They can live in any component.
Wheel axle centers are found automatically - no markers needed for wheels.

Run it from: Utilities -> ADD-INS -> Scripts and Add-Ins -> (+) -> Script
from device -> pick this folder -> Run.
"""

import adsk.core
import adsk.fusion
import traceback
import os
import json
import struct
import fnmatch
import time

# ---- edit this ----

MODE = "export"        # "dump" or "export"

OUTPUT_DIR = r"D:\rover_export"     # WSL path: /mnt/d/rover_export

# link -> patterns, matched against the occurrence name ("Motorhub V4:1") or
# its full path ("Suspension (L) V5:1/Motorhub V4:1"). * and ?, case-insensitive.
# a match takes everything under it unless something deeper matches another link.
# unmatched -> base_link. __ignore__ -> not exported.
LINKS = {
    # "Bogie Link V4" belongs to the rocker, "RB Assembly V4" to the bogie
    "rocker_left": ["Suspension (L)*/PVC Rocker:1", "Suspension (L)*/Rocker Assembly V4:1",
                    "Suspension (L)*/PVC R -> B:1", "Suspension (L)*/Bogie Link V4:1",
                    "Suspension (L)*/Motorhub V4:3"],
    "bogie_left":  ["Suspension (L)*/RB Assembly V4:1", "Suspension (L)*/PVC Bogie:*",
                    "Suspension (L)*/Motorhub V4:1", "Suspension (L)*/Motorhub V4:2"],
    "rocker_right": ["Suspension (R)*/PVC Rocker:1", "Suspension (R)*/Rocker Assembly V4:1",
                     "Suspension (R)*/PVC R -> B:1", "Suspension (R)*/Bogie Link V4:1",
                     "Suspension (R)*/Motorhub V4:1"],
    "bogie_right":  ["Suspension (R)*/RB Assembly V4:1", "Suspension (R)*/PVC Bogie:*",
                     "Suspension (R)*/Motorhub V4:2", "Suspension (R)*/Motorhub V4:3"],
    "wheel_front_left":   ["Suspension (L)*/TPU Wheel Assembly:2"],
    "wheel_middle_left":  ["Suspension (L)*/TPU Wheel Assembly:1"],
    "wheel_rear_left":    ["Suspension (L)*/TPU Wheel Assembly:3"],
    "wheel_front_right":  ["Suspension (R)*/TPU Wheel Assembly:3"],
    "wheel_middle_right": ["Suspension (R)*/TPU Wheel Assembly:2"],
    "wheel_rear_right":   ["Suspension (R)*/TPU Wheel Assembly:1"],
    # diff rods are handled by the RockerDifferential plugin
    "__ignore__": ["Rod End M8*", "DiffBarConnectorV5*"],
}

SKIP_HIDDEN = True      
MESH_QUALITY = "low"      # "low" or "medium"

# -------------------


def _mat_from_fusion(m3d):
    """Fusion Matrix3D -> 4x4 nested list (row-major, translation in cm)."""
    a = m3d.asArray()
    return [list(a[0:4]), list(a[4:8]), list(a[8:12]), list(a[12:16])]


def _matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


_IDENT = [[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]


def mesh_mass_props(verts, tris):
    """Volume, centroid and second moments of a closed triangle mesh
    (unit density), via signed tetrahedra. verts: list of (x,y,z).
    Returns (vol, [cx,cy,cz], [[Ixx..]]) with inertia about the centroid."""
    vol = 0.0
    cx = cy = cz = 0.0
    # integrals of x^2, y^2, z^2, xy, yz, zx
    xx = yy = zz = xy = yz = zx = 0.0
    for (i, j, k) in tris:
        x1, y1, z1 = verts[i]
        x2, y2, z2 = verts[j]
        x3, y3, z3 = verts[k]
        d = (x1 * (y2 * z3 - y3 * z2) - x2 * (y1 * z3 - y3 * z1) + x3 * (y1 * z2 - y2 * z1))
        v = d / 6.0
        vol += v
        cx += v * (x1 + x2 + x3) / 4.0
        cy += v * (y1 + y2 + y3) / 4.0
        cz += v * (z1 + z2 + z3) / 4.0
        f = d / 60.0
        xx += f * (x1*x1 + x2*x2 + x3*x3 + x1*x2 + x2*x3 + x3*x1)
        yy += f * (y1*y1 + y2*y2 + y3*y3 + y1*y2 + y2*y3 + y3*y1)
        zz += f * (z1*z1 + z2*z2 + z3*z3 + z1*z2 + z2*z3 + z3*z1)
        f2 = d / 120.0
        xy += f2 * (2*x1*y1 + 2*x2*y2 + 2*x3*y3 + x1*y2 + x2*y1 + x1*y3 + x3*y1 + x2*y3 + x3*y2)
        yz += f2 * (2*y1*z1 + 2*y2*z2 + 2*y3*z3 + y1*z2 + y2*z1 + y1*z3 + y3*z1 + y2*z3 + y3*z2)
        zx += f2 * (2*z1*x1 + 2*z2*x2 + 2*z3*x3 + z1*x2 + z2*x1 + z1*x3 + z3*x1 + z2*x3 + z3*x2)
    if abs(vol) < 1e-15:
        return 0.0, [0.0, 0.0, 0.0], [[0.0] * 3 for _ in range(3)]
    if vol < 0:  # inward-facing normals: flip everything
        vol, cx, cy, cz = -vol, -cx, -cy, -cz
        xx, yy, zz, xy, yz, zx = -xx, -yy, -zz, -xy, -yz, -zx
    c = [cx / vol, cy / vol, cz / vol]
    # inertia about origin
    Io = [[yy + zz, -xy, -zx], [-xy, xx + zz, -yz], [-zx, -yz, xx + yy]]
    # parallel axis to centroid
    I = _shift_inertia(Io, vol, c, toward_com=True)
    return vol, c, I


def _shift_inertia(I, m, c, toward_com):
    """Parallel-axis theorem. toward_com=True: I is about origin -> about c."""
    s = -1.0 if toward_com else 1.0
    x, y, z = c
    d = [[y*y + z*z, -x*y, -x*z], [-x*y, x*x + z*z, -y*z], [-x*z, -y*z, x*x + y*y]]
    return [[I[i][j] + s * m * d[i][j] for j in range(3)] for i in range(3)]


def combine(parts):
    """parts: list of (mass, com, I_about_com). Returns combined (m, com, I)."""
    m = sum(p[0] for p in parts)
    if m <= 0:
        return 0.0, [0.0, 0.0, 0.0], [[0.0] * 3 for _ in range(3)]
    com = [sum(p[0] * p[1][i] for p in parts) / m for i in range(3)]
    I = [[0.0] * 3 for _ in range(3)]
    for (mi, ci, Ii) in parts:
        r = [ci[k] - com[k] for k in range(3)]
        Ish = _shift_inertia(Ii, mi, r, toward_com=False)
        for a in range(3):
            for b in range(3):
                I[a][b] += Ish[a][b]
    return m, com, I


def write_binary_stl(path, verts, tris, name="link"):
    with open(path, "wb") as f:
        header = ("ExportRoverLinks " + name).encode("ascii", "ignore")[:80]
        f.write(header + b" " * (80 - len(header)))
        f.write(struct.pack("<I", len(tris)))
        for (i, j, k) in tris:
            f.write(struct.pack("<3f", 0.0, 0.0, 0.0))  # normal (viewers recompute)
            for idx in (i, j, k):
                f.write(struct.pack("<3f", *verts[idx]))
            f.write(b"\x00\x00")


class Exporter:
    def __init__(self, app):
        self.app = app
        self.design = adsk.fusion.Design.cast(app.activeProduct)
        self.root = self.design.rootComponent
        self.patterns = {ln: [p.lower() for p in pats] for ln, pats in LINKS.items()}
        self.mesh_cache = {}   # native body entityToken -> (verts_cm, tris)
        self.links = {}        # link -> {"verts": [], "tris": [], "parts": [], "bodies": []}
        self.tree_lines = []
        self.warnings = []
        self.points = {}       # construction points named joint_* -> world xyz (m)
        self.stats = {"direct": 0, "composed": 0, "unverified": 0, "mismatch": 0}

    # ---------- matching ----------
    def match(self, occ_name, path=""):
        n = occ_name.lower()
        full = (path.strip("/") + "/" + occ_name).strip("/").lower()
        for link, pats in self.patterns.items():
            for p in pats:
                if fnmatch.fnmatchcase(n, p) or fnmatch.fnmatchcase(full, p):
                    return link
        return None

    # ---------- meshing ----------
    def body_mesh(self, body):
        key = body.entityToken
        if key in self.mesh_cache:
            return self.mesh_cache[key]
        calc = body.meshManager.createMeshCalculator()
        q = adsk.fusion.TriangleMeshQualityOptions
        calc.setQuality(q.NormalQualityTriangleMesh if MESH_QUALITY == "medium"
                        else q.LowQualityTriangleMesh)
        mesh = calc.calculate()
        c = mesh.nodeCoordinatesAsDouble
        verts = [(c[i], c[i + 1], c[i + 2]) for i in range(0, len(c), 3)]
        idx = mesh.nodeIndices
        tris = [(idx[i], idx[i + 1], idx[i + 2]) for i in range(0, len(idx), 3)]
        self.mesh_cache[key] = (verts, tris)
        return verts, tris

    def _apply(self, M, verts_cm):
        out = []
        for (x, y, z) in verts_cm:
            out.append((M[0][0] * x + M[0][1] * y + M[0][2] * z + M[0][3],
                        M[1][0] * x + M[1][1] * y + M[1][2] * z + M[1][3],
                        M[2][0] * x + M[2][1] * y + M[2][2] * z + M[2][3]))
        return out

    @staticmethod
    def _bbox(pts):
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]
        return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))

    def _matches_fusion(self, pts, body):
        """Does the transformed mesh sit where Fusion says this body is?"""
        try:
            bb = body.boundingBox  # world-space for proxy bodies (cm)
            fmin = (bb.minPoint.x, bb.minPoint.y, bb.minPoint.z)
            fmax = (bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z)
        except Exception:
            return None
        lo, hi = self._bbox(pts)
        diag = sum((fmax[i] - fmin[i]) ** 2 for i in range(3)) ** 0.5
        tol = 0.2 + 0.03 * diag
        for i in range(3):
            if abs((lo[i] + hi[i]) / 2 - (fmin[i] + fmax[i]) / 2) > tol:
                return False
            if abs((hi[i] - lo[i]) - (fmax[i] - fmin[i])) > tol:
                return False
        return True

    def place_body(self, link, body, cands, path):
        """cands: [(label, matrix), ...] in preference order. Meshes the native
        body, picks the candidate transform that matches Fusion's own world
        bounding box, adds it to the link. Returns the label that matched."""
        if SKIP_HIDDEN and not body.isLightBulbOn:
            return None
        if not body.isSolid:
            self.warnings.append("Skipped non-solid (surface) body: %s/%s" % (path, body.name))
            return None
        native = body.nativeObject if body.nativeObject else body
        try:
            verts_cm, tris = self.body_mesh(native)
        except Exception:
            self.warnings.append("Could not mesh body: %s/%s" % (path, body.name))
            return None
        chosen, pts = None, None
        for label, M in cands:
            p = self._apply(M, verts_cm)
            ok = self._matches_fusion(p, body)
            if ok is None:                      # can't verify: take first candidate
                chosen, pts = label, p
                self.stats["unverified"] += 1
                break
            if ok:
                chosen, pts = label, p
                self.stats[label] += 1
                break
        if chosen is None:
            chosen, pts = cands[0][0], self._apply(cands[0][1], verts_cm)
            self.stats["mismatch"] += 1
            self.warnings.append("Position check FAILED (kept best guess): %s/%s" % (path, body.name))
        if link != "__ignore__":
            self.add_mesh(link, native, [(x / 100.0, y / 100.0, z / 100.0) for (x, y, z) in pts], tris, path, body)
        return chosen

    def add_mesh(self, link, native, vw, tris, path, body):
        try:
            mass = native.physicalProperties.mass  # kg
            mat = native.material.name if native.material else "?"
        except Exception:
            mass, mat = 0.0, "?"
        vol, com, I1 = mesh_mass_props(vw, tris)
        L = self.links.setdefault(link, {"verts": [], "tris": [], "parts": [], "bodies": []})
        off = len(L["verts"])
        L["verts"].extend(vw)
        L["tris"].extend((a + off, b + off, c + off) for (a, b, c) in tris)
        if vol > 0 and mass > 0:
            dens = mass / vol
            I = [[I1[i][j] * dens for j in range(3)] for i in range(3)]
            L["parts"].append((mass, com, I))
        elif mass > 0:
            self.warnings.append("Open/broken mesh, mass kept at its bounding center: %s/%s" % (path, body.name))
            lo, hi = self._bbox(vw)
            com = [(lo[i] + hi[i]) / 2 for i in range(3)]
            L["parts"].append((mass, com, [[0.0] * 3 for _ in range(3)]))
        L["bodies"].append({"path": path + "/" + body.name, "mass_kg": round(mass, 5), "material": mat})

    # ---------- traversal ----------
    def collect_points(self, comp, world, path):
        """Record construction points named joint_* (pivot markers)."""
        try:
            pts = comp.constructionPoints
        except Exception:
            return
        for cp in pts:
            if not cp.name.lower().startswith("joint_"):
                continue
            g = cp.geometry
            (X, Y, Z), = self._apply(world, [(g.x, g.y, g.z)])
            if cp.name in self.points:
                self.warnings.append("Duplicate construction point name %s (in %s) - keeping first"
                                     % (cp.name, path or "root"))
                continue
            self.points[cp.name] = [X / 100.0, Y / 100.0, Z / 100.0]

    def walk_root(self):
        self.collect_points(self.root, _IDENT, "")
        for body in self.root.bRepBodies:
            self.place_body("base_link", body, [("direct", _IDENT)], "")
        self._walk_occs(self.root.occurrences, _IDENT, "base_link", "", 0)

    def _walk_occs(self, occs, parent_abs, link, path, depth):
        """Walk PROXY occurrences. Fusion's transform2 on a nested proxy may be
        absolute or parent-relative depending on context, so both readings are
        tried and each body is checked against Fusion's own bounding box."""
        for occ in occs:
            if SKIP_HIDDEN and not occ.isLightBulbOn:
                self.tree_lines.append("  " * (depth + 1) + occ.name + "   [hidden - skipped]")
                continue
            m = self.match(occ.name, path)
            sub_link = m if m else link
            tag = ("   -> " + m) if m else ""
            T = _mat_from_fusion(occ.transform2)
            direct, composed = T, _matmul(parent_abs, T)
            pref = "composed" if self.stats["composed"] > self.stats["direct"] else "direct"
            cands = [("direct", direct), ("composed", composed)]
            if pref == "composed":
                cands.reverse()
            occ_abs = dict(cands)[pref]
            nb = 0
            for body in occ.bRepBodies:
                nb += 1
                got = self.place_body(sub_link, body, cands, path + "/" + occ.name)
                if got:
                    occ_abs = dict(cands)[got]
            self.tree_lines.append("  " * (depth + 1) + occ.name + "   (%d bodies)%s" % (nb, tag))
            self.collect_points(occ.component, occ_abs, path + "/" + occ.name)
            self._walk_occs(occ.childOccurrences, occ_abs, sub_link, path + "/" + occ.name, depth + 1)

    def run(self):
        t0 = time.time()
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        self.tree_lines.append("ROOT: " + self.root.name + "   (%d bodies)" % self.root.bRepBodies.count)
        if MODE == "dump":
            self.centers = {}
            for o in self.root.allOccurrences:
                try:
                    bb = o.boundingBox
                    self.centers[o.fullPathName] = [(bb.minPoint.x + bb.maxPoint.x) * 5.0,
                                                    (bb.minPoint.y + bb.maxPoint.y) * 5.0,
                                                    (bb.minPoint.z + bb.maxPoint.z) * 5.0]
                except Exception:
                    pass
            self.tree_lines[0] += "        [center x, y, z in mm, Fusion axes]"
            self._dump_only(self.root, 0, _IDENT, "", "")
            self.tree_lines.append("")
            self.tree_lines.append("Pivot markers (construction points named joint_*):")
            for n, xyz in sorted(self.points.items()):
                self.tree_lines.append("  %-28s %8.1f %8.1f %8.1f mm" % (n, xyz[0]*1000, xyz[1]*1000, xyz[2]*1000))
            if not self.points:
                self.tree_lines.append("  (none yet)")
            p = os.path.join(OUTPUT_DIR, "rover_tree.txt")
            with open(p, "w", encoding="utf-8") as f:
                f.write("\n".join(self.tree_lines))
            return "Tree written to:\n" + p + "\n\nUpload rover_tree.txt to Claude, or edit LINKS and set MODE = \"export\"."

        self.walk_root()
        mdir = os.path.join(OUTPUT_DIR, "meshes")
        os.makedirs(mdir, exist_ok=True)
        out = {"units": "meters, kg, kg*m^2", "frame": "Fusion world frame (as modeled)",
               "fusion_document": self.app.activeDocument.name, "links": {}}
        for link, L in self.links.items():
            if not L["tris"]:
                continue
            write_binary_stl(os.path.join(mdir, link + ".stl"), L["verts"], L["tris"], link)
            m, com, I = combine(L["parts"])
            xs = [v[0] for v in L["verts"]]; ys = [v[1] for v in L["verts"]]; zs = [v[2] for v in L["verts"]]
            out["links"][link] = {
                "mass": m, "com": com, "inertia": I,
                "bbox_min": [min(xs), min(ys), min(zs)], "bbox_max": [max(xs), max(ys), max(zs)],
                "triangles": len(L["tris"]), "num_bodies": len(L["bodies"]), "bodies": L["bodies"],
            }
        missing = [ln for ln in LINKS if ln != "__ignore__" and ln not in out["links"]]
        if missing:
            self.warnings.append("These links matched NOTHING (check patterns): " + ", ".join(missing))
        out["points"] = self.points
        out["position_check"] = self.stats
        out["warnings"] = self.warnings
        with open(os.path.join(OUTPUT_DIR, "links.json"), "w", encoding="utf-8") as f:
            json.dump(out, f, indent=1)
        with open(os.path.join(OUTPUT_DIR, "rover_tree.txt"), "w", encoding="utf-8") as f:
            f.write("\n".join(self.tree_lines))
        summary = ["Exported %d links in %.0f s to:\n%s\n" % (len(out["links"]), time.time() - t0, OUTPUT_DIR)]
        for ln, d in out["links"].items():
            summary.append("  %-20s %7.2f kg  %4d bodies" % (ln, d["mass"], d["num_bodies"]))
        summary.append("\nPivot markers found: " + (", ".join(sorted(self.points)) or "NONE"))
        st = self.stats
        summary.append("Position check: %d bodies OK (%d direct, %d composed), %d unverified, %d FAILED"
                       % (st["direct"] + st["composed"], st["direct"], st["composed"], st["unverified"], st["mismatch"]))
        if self.warnings:
            summary.append("\n%d warning(s) - see links.json" % len(self.warnings))
        return "\n".join(summary)

    def _dump_only(self, comp, depth, world, path, fpath):
        self.collect_points(comp, world, "")
        for occ in comp.occurrences:
            hidden = "" if occ.isLightBulbOn else "   [hidden]"
            m = self.match(occ.name, path)
            tag = ("   -> " + m) if m else ""
            fp = (fpath + "+" + occ.name) if fpath else occ.name
            c = self.centers.get(fp)
            pos = ("   [%.0f, %.0f, %.0f]" % tuple(c)) if c else ""
            self.tree_lines.append("  " * (depth + 1) + occ.name + "   (%d bodies)%s%s%s"
                                   % (occ.component.bRepBodies.count, pos, tag, hidden))
            self._dump_only(occ.component, depth + 1, _matmul(world, _mat_from_fusion(occ.transform2)),
                            (path + "/" + occ.name).strip("/"), fp)


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        if not adsk.fusion.Design.cast(app.activeProduct):
            ui.messageBox("Open your rover design (Design workspace) first.")
            return
        msg = Exporter(app).run()
        ui.messageBox(msg, "ExportRoverLinks")
    except Exception:
        if ui:
            ui.messageBox("Failed:\n{}".format(traceback.format_exc()))
