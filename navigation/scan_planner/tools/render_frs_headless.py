#!/usr/bin/env python3
"""MuJoCo EGL rendering of recorded ROS kinematics; never a dynamics replay.

Uses the real Slash mesh and the same five-bar height kinematics as ROS.
No mj_step is called. Pedestrians and FRS are drawn from the recorded experiment.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[1] / "src/planner/plan_manage/scripts"))
import leg_height_control as legs  # noqa: E402
from validate_dynamic_navigation import pedestrian_at  # noqa: E402


class ReplayRenderer:
    def __init__(self, run, slash_root, width=1600, height=1000):
        self.run = run
        self.trace = json.loads((run / "trace.json").read_text())
        self.result = json.loads((run / "result.json").read_text())
        self.model = mujoco.MjModel.from_xml_path(
            str(slash_root / "assets/mjcf/scene_flat.xml")
        )
        self.model.vis.quality.shadowsize = 2048
        self.model.vis.quality.offsamples = 4
        self.model.light_diffuse[:] *= 0.65
        self.model.light_ambient[:] *= 0.7
        self.model.vis.headlight.diffuse[:] = 0.15
        self.model.vis.headlight.ambient[:] = 0.15
        self.model.geom_matid[self.model.geom("floor").id] = -1
        self.model.geom_rgba[self.model.geom("floor").id] = [0.90, 0.92, 0.94, 1]
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [0, -0.15, 0.35]
        self.camera.distance = 6.7
        self.camera.azimuth = 135
        self.camera.elevation = -42
        self.option = mujoco.MjvOption()
        self.option.geomgroup[3] = 0
        self.width = width
        self.height = height
        self.font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", max(14, width // 65)
        )
        self.colors = [
            (0.83, 0.22, 0.20, 1),
            (0.47, 0.30, 0.74, 1),
            (0.90, 0.55, 0.12, 1),
        ]

    def geom(self, kind, size, pos, rgba):
        scene = self.renderer.scene
        g = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            g,
            kind,
            np.array(size, dtype=float),
            np.array(pos, dtype=float),
            np.eye(3).ravel(),
            np.array(rgba, dtype=np.float32),
        )
        scene.ngeom += 1

    def box(self, center, half, color):
        self.geom(mujoco.mjtGeom.mjGEOM_BOX, half, center, color)

    def frame(self, t, annotate=True):
        pose = min(self.trace["poses"], key=lambda row: abs(row["t"] - t))
        decision = min(self.trace["decisions"], key=lambda row: abs(row["t"] - t))
        x, y, z, yaw = pose["pose"]
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        angle = (
            yaw - math.pi / 2
        )  # Mesh forward is local +Y; ROS navigation forward is +X.
        self.data.qpos[:7] = [x, y, z, math.cos(angle / 2), 0, 0, math.sin(angle / 2)]
        h = z - 0.0322
        for side, ik, conv in [
            ("l", legs.left_ik, legs.firmware_to_mujoco_left),
            ("r", legs.right_ik, legs.firmware_to_mujoco_right),
        ]:
            a, b, _ = ik(0, h, 0)
            f, b = conv(a, b)
            f2, b2 = legs.five_bar_knee_angles_closed(f, b)
            # MJCF rf knee axis is reversed relative to the ROS visualization convention.
            if side == "r":
                f2 = -f2
            for name, value in [
                (side + "f_joint1", f),
                (side + "b_joint1", b),
                (side + "f_joint2", f2),
                (side + "b_joint2", b2),
            ]:
                self.data.joint(name).qpos[0] = value
        mujoco.mj_forward(self.model, self.data)
        if np.max(np.abs(self.data.efc_pos[:6])) > 1e-5:
            raise ValueError("Rendered leg closed-chain geometry is inconsistent")
        self.renderer.update_scene(
            self.data, camera=self.camera, scene_option=self.option
        )
        # Local crop of the same static wall and lintel used by the ROS scene.
        clearance = 0.62 if self.result["case"] == "combined" else 0.8
        for xx in [-2, 2]:
            self.box([xx, 0, 0.65], [1, 0.10, 0.65], [0.60, 0.66, 0.71, 1])
        self.box([0, 0, clearance + 0.08], [1, 0.30, 0.08], [0.35, 0.47, 0.56, 0.80])
        for k in range(-3, 4):
            self.box([k, 0, 0.002], [0.005, 3, 0.001], [0.73, 0.77, 0.80, 1])
            self.box([0, k, 0.002], [3, 0.005, 0.001], [0.73, 0.77, 0.80, 1])
        # Goal marker on the ground.
        self.geom(
            mujoco.mjtGeom.mjGEOM_CYLINDER,
            [0.16, 0.005, 0],
            [0, 2, 0.01],
            [0.15, 0.50, 0.25, 1],
        )
        for track in decision.get("tracks", []):
            identity = int(track["id"].split("_")[-1])
            color = self.colors[identity % 3]
            for index, alpha in [(19, 0.14), (9, 0.24)]:
                _, cx, cy, hx, hy = track["boxes"][index]
                # Raw position FRS only; no body/margin inflation is drawn here.
                if hx > 1e-6 and hy > 1e-6:
                    self.box([cx, cy, 0.012], [hx, hy, 0.005], [*color[:3], alpha])
        count = self.result.get("pedestrians", 1)
        for identity in range(count):
            p, _ = pedestrian_at(
                pose["t"], self.result["seed"], self.result["case"], identity, count
            )
            color = self.colors[identity % 3]
            self.geom(
                mujoco.mjtGeom.mjGEOM_CYLINDER, [0.17, 0.38, 0], [*p, 0.98], color
            )
            self.geom(
                mujoco.mjtGeom.mjGEOM_SPHERE,
                [0.14, 0, 0],
                [*p, 1.52],
                [0.78, 0.61, 0.47, 1],
            )
            for dx in [-0.09, 0.09]:
                self.geom(
                    mujoco.mjtGeom.mjGEOM_CAPSULE,
                    [0.065, 0.23, 0],
                    [p[0] + dx, p[1], 0.35],
                    [0.23, 0.27, 0.33, 1],
                )
            self.geom(
                mujoco.mjtGeom.mjGEOM_CYLINDER,
                [0.30, 0.004, 0],
                [*p, 0.007],
                [*color[:3], 0.35],
            )
        # Render the actual past trajectory, not a designed illustration.
        points = [row["pose"][:2] for row in self.trace["poses"] if row["t"] <= t][::40]
        for xx, yy in points:
            self.geom(
                mujoco.mjtGeom.mjGEOM_SPHERE,
                [0.025, 0, 0],
                [xx, yy, 0.025],
                [0.05, 0.35, 0.75, 1],
            )
        img = Image.fromarray(self.renderer.render())
        if not annotate:
            return img
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, self.width, 72], fill="white")
        draw.text(
            (18, 8),
            f"{self.result['mode'].upper()} | t={pose['t']:.1f} s | base z={z:.3f} m | speed={pose['v']:.2f} m/s",
            font=self.font,
            fill="#152d44",
        )
        draw.text(
            (18, 38),
            f"ROS kinematic replay / MuJoCo EGL | scale={decision['scale']:.1f} | raw FRS: 1 s and 2 s",
            font=self.font,
            fill="#41576c",
        )
        draw.rectangle([0, self.height - 36, self.width, self.height], fill="white")
        draw.text(
            (18, self.height - 32),
            "1 m floor grid | blue dots: executed path | targets: red / purple / orange",
            font=self.font,
            fill="#41576c",
        )
        return img


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument(
        "--slash-root", type=Path, default=HERE.parents[4] / "slash_robot_control"
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--times", type=float, nargs="+", default=[4, 16, 28, 40])
    parser.add_argument("--animation", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    replay = ReplayRenderer(args.run, args.slash_root)
    manifest = dict(
        source=str(args.run.resolve()),
        kind="kinematic log replay; no mj_step/dynamics",
        source_sha256=hashlib.sha256(
            (args.run / "trace.json").read_bytes()
        ).hexdigest(),
        timestamps=args.times,
        backend=os.environ["MUJOCO_GL"],
        camera=dict(azimuth=135, elevation=-42, distance=6.7),
    )
    for t in args.times:
        if t > replay.trace["poses"][-1]["t"]:
            raise ValueError("Requested frame exceeds recorded run")
        replay.frame(t).save(args.output / f"frame_{t:g}s.png", dpi=(300, 300))
    replay.renderer.close()
    if args.animation:
        replay = ReplayRenderer(args.run, args.slash_root, width=960, height=600)
        frames = [
            replay.frame(t) for t in np.arange(0, replay.trace["poses"][-1]["t"], 0.25)
        ]
        frames[0].save(
            args.output / "replay.gif",
            save_all=True,
            append_images=frames[1:],
            duration=250,
            loop=0,
        )
        replay.renderer.close()
    (args.output / "render_manifest.json").write_text(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
