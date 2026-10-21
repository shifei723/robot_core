#!/usr/bin/env python3
"""Paired single/multiple-pedestrian ROS experiment with causal predictions.

The pedestrian is an explicit ideal tracker input, not a LiDAR detector.
Future truth is used only by the evaluator. Each run owns an isolated ROS domain.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
from validate_height_navigation import geometry


def pedestrian(t, seed, case):
    rng = np.random.default_rng(seed)
    start = -2.2 + rng.uniform(-0.15, 0.15)
    speed = 0.32 + rng.uniform(-0.03, 0.03)
    # Acceleration onset is independent of the robot/method, then speed holds.
    elapsed = max(t - 3.0, 0.0)
    ramp = min(elapsed, 2.0)
    acceleration = 0.12
    x = start + speed * t + 0.5 * acceleration * ramp**2
    x += acceleration * 2.0 * max(elapsed - 2.0, 0.0)
    vx = speed + acceleration * ramp
    y = 0.0 if case == "combined" else -0.8
    return np.array([x, y]), np.array([vx, 0.0])


def pedestrian_at(t, seed, case, identity=0, count=1):
    if count == 1:
        return pedestrian(t, seed, case)
    delay = [0.0, 8.0, 16.0][identity]
    p, v = pedestrian(max(0.0, t - delay), seed + 1000 * identity, case)
    if t < delay:
        v[:] = 0
    if identity % 2:
        p[0] *= -1
        v[0] *= -1
    # Cross on each side of the doorway, never through the solid dividing wall.
    p[1] = ([-0.65, 0.65, 1.35] if case == "combined" else [-0.8, 0.45, 1.25])[identity]
    return p, v


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mode", choices=["cv", "frs", "physical"], required=True)
    p.add_argument("--case", choices=["crossing", "combined"], default="crossing")
    p.add_argument("--seed", type=int, default=100)
    p.add_argument("--domain", type=int, default=151)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--timeout", type=float, default=75)
    p.add_argument("--pedestrians", type=int, choices=[1, 3], default=1)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from scan_planner_msgs.msg import Bspline

    rclpy.init()
    node = rclpy.create_node("dynamic_navigation_validator")
    track_pub = node.create_publisher(Odometry, "/dynamic_navigation/pedestrian", 10)
    goal_pub = node.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
    data = dict(pose=None, clouds=0, plans=0, v=0.0, w=0.0, state="")
    poses, decisions, plans, observations = [], [], [], []
    origin = None

    def odom(msg):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        data["pose"] = [p.x, p.y, p.z, yaw]
        if origin is not None:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            poses.append(
                dict(
                    t=stamp - origin,
                    pose=data["pose"],
                    v=data["v"],
                    w=data["w"],
                    state=data["state"],
                )
            )

    def command(msg):
        data.update(v=msg.linear.x, w=msg.angular.z)

    def decision(msg):
        row = json.loads(msg.data)
        if origin is not None:
            row["t"] = row.pop("stamp") - origin
            decisions.append(row)

    def state(msg):
        data["state"] = msg.data

    def cloud(msg):
        data["clouds"] += 1

    def spline(msg):
        data["plans"] += 1
        plans.append(
            dict(knots=list(msg.knots), points=[[p.x, p.y, p.z] for p in msg.pos_pts])
        )

    _subscriptions = [
        node.create_subscription(
            Odometry, "/quad_0/body_pose", odom, qos_profile_sensor_data
        ),
        node.create_subscription(Twist, "/quad_0/cmd_vel", command, 10),
        node.create_subscription(String, "/dynamic_navigation/decision", decision, 10),
        node.create_subscription(String, "/height_control/state", state, 10),
        node.create_subscription(
            PointCloud2, "/quad_0/cloud", cloud, qos_profile_sensor_data
        ),
        node.create_subscription(Bspline, "/planning/bspline", spline, 10),
    ]
    clearance = 0.62 if args.case == "combined" else 0.80
    log = (args.output / "launch.log").open("w")
    proc = subprocess.Popen(
        [
            "ros2",
            "launch",
            "scan_planner",
            "run.launch.py",
            "robot_model:=slash",
            "is_real_world:=false",
            "controller_mode:=closed_loop",
            "use_gpu:=false",
            "dynamic_obstacles:=false",
            f"dynamic_prediction:={args.mode}",
            f"dynamic_min_tracks:={args.pedestrians}",
            "init_x:=0.0",
            "init_y:=-2.0",
            f"init_yaw:={math.pi / 2}",
            f"beam_clearance:={clearance}",
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    outcome = dict(
        mode=args.mode,
        case=args.case,
        seed=args.seed,
        pedestrians=args.pedestrians,
        reached=False,
        error=None,
    )
    last_index = -1
    try:
        begin = time.monotonic()
        while time.monotonic() - begin < 25:
            rclpy.spin_once(node, timeout_sec=0.02)
            if (
                data["pose"] is not None
                and data["clouds"] > 15
                and goal_pub.get_subscription_count()
                and time.monotonic() - begin > 4
            ):
                break
        else:
            raise RuntimeError("Startup timeout")
        origin = node.get_clock().now().nanoseconds * 1e-9
        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.y = 2.0
        goal.pose.position.z = 0.28
        goal.pose.orientation.w = 1.0
        goal_pub.publish(goal)
        settled = None
        while True:
            now = node.get_clock().now()
            t = now.nanoseconds * 1e-9 - origin
            if t > args.timeout:
                break
            index = int(t / 0.1)
            if index > last_index:
                sample_t = index * 0.1
                # Reverse arrival order every other frame to exercise identity isolation.
                identities = list(range(args.pedestrians))
                if index % 2:
                    identities.reverse()
                for identity in identities:
                    pos, vel = pedestrian_at(
                        sample_t, args.seed, args.case, identity, args.pedestrians
                    )
                    rng = np.random.default_rng(
                        [10000 + args.seed, index]
                        + ([identity] if args.pedestrians > 1 else [])
                    )
                    measured_pos = pos + rng.normal(0, 0.002, 2)
                    measured_vel = vel + rng.normal(0, 0.01, 2)
                    msg = Odometry()
                    msg.header.frame_id = "world"
                    msg.child_frame_id = f"pedestrian_{identity}"
                    msg.header.stamp = rclpy.time.Time(
                        seconds=origin + sample_t
                    ).to_msg()
                    msg.pose.pose.position.x, msg.pose.pose.position.y = map(
                        float, measured_pos
                    )
                    msg.twist.twist.linear.x, msg.twist.twist.linear.y = map(
                        float, measured_vel
                    )
                    track_pub.publish(msg)
                    observations.append(
                        dict(
                            t=sample_t,
                            index=index,
                            id=msg.child_frame_id,
                            position=measured_pos.tolist(),
                            velocity=measured_vel.tolist(),
                        )
                    )
                last_index = index
            rclpy.spin_once(node, timeout_sec=0.005)
            x, y, z, yaw = data["pose"]
            if math.hypot(x, y - 2) < 0.15 and abs(data["v"]) + abs(data["w"]) < 0.05:
                settled = t if settled is None else settled
                if t - settled > 1:
                    outcome["reached"] = True
                    break
            else:
                settled = None
            if proc.poll() is not None:
                raise RuntimeError("Simulation exited")
        if not decisions or not poses or not data["plans"]:
            raise RuntimeError("Missing decisions/odometry/B-spline")
        if any(b["t"] <= a["t"] for a, b in zip(decisions, decisions[1:])):
            raise RuntimeError(
                "Nonmonotonic decision timestamps; cannot evaluate prediction alignment"
            )
        times = np.array([r["t"] for r in poses])
        xy = np.array([r["pose"][:2] for r in poses])
        gaps = []
        per_obstacle_gaps = [[] for _ in range(args.pedestrians)]
        beam = []
        collisions = 0
        # Interpolate at <=5 ms to catch between-message contacts.
        for a, b in zip(poses, poses[1:]):
            count = max(1, math.ceil((b["t"] - a["t"]) / 0.005))
            for f in np.linspace(0, 1, count + 1):
                t = a["t"] + f * (b["t"] - a["t"])
                pose = np.array(a["pose"]) + f * (np.array(b["pose"]) - a["pose"])
                # Exact union of two robot discs (0.25 m, offset +/-0.18) vs pedestrian disc 0.30.
                direction = np.array([math.cos(pose[3]), math.sin(pose[3])])
                for identity in range(args.pedestrians):
                    human, _ = pedestrian_at(
                        t, args.seed, args.case, identity, args.pedestrians
                    )
                    gap = min(
                        np.linalg.norm(pose[:2] + s * 0.18 * direction - human) - 0.55
                        for s in [-1, 1]
                    )
                    gaps.append(gap)
                    per_obstacle_gaps[identity].append(gap)
                solid, under, margin = geometry(*pose, clearance)
                collisions += int(solid)
                if under:
                    beam.append(margin)
        cover = {}
        area = {}
        center_error = {}
        for horizon in [0.5, 1.0, 2.0, 3.0]:
            checks = []
            sizes = []
            errors = []
            for r in decisions:
                # Fixed encounter interval prevents slower methods receiving more easy samples.
                if r["stale"] or not 0 <= r["t"] <= (
                    30 if args.pedestrians > 1 else 10
                ):
                    continue
                for track in r["tracks"]:
                    identity = int(track["id"].split("_")[-1])
                    box = track["boxes"][round(horizon / 0.1) - 1]
                    truth, _ = pedestrian_at(
                        r["t"] + horizon,
                        args.seed,
                        args.case,
                        identity,
                        args.pedestrians,
                    )
                    checks.append(
                        bool(
                            np.all(
                                np.abs(truth - box[1:3]) <= np.array(box[3:5]) + 1e-9
                            )
                        )
                    )
                    sizes.append(4 * box[3] * box[4])
                    errors.append(float(np.linalg.norm(truth - box[1:3])))
            cover[str(horizon)] = float(np.mean(checks)) if checks else None
            area[str(horizon)] = float(np.mean(sizes)) if sizes else None
            center_error[str(horizon)] = float(np.mean(errors)) if errors else None
        decision_dt = np.diff([r["t"] for r in decisions] + [times[-1]])
        outcome.update(
            duration_s=float(times[-1]),
            min_clearance_m=float(min(gaps)),
            collision=bool(min(gaps) < 0),
            static_collision_samples=collisions,
            min_beam_margin_m=min(beam) if beam else None,
            min_base_height_m=min(r["pose"][2] for r in poses),
            path_length_m=float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()),
            prediction_coverage=cover,
            prediction_area_m2=area,
            prediction_center_error_m=center_error,
            prediction_evaluation_interval_s=[0, 30 if args.pedestrians > 1 else 10],
            min_clearance_by_id={
                f"pedestrian_{i}": float(min(g))
                for i, g in enumerate(per_obstacle_gaps)
            },
            observed_track_counts=sorted(set(r["track_count"] for r in decisions)),
            limiting_obstacle_samples={
                f"pedestrian_{i}": sum(
                    r["nominal_obstacle"] == f"pedestrian_{i}" for r in decisions
                )
                for i in range(args.pedestrians)
            },
            wait_s=float(
                sum(
                    max(0, dt)
                    for r, dt in zip(decisions, decision_dt)
                    if r["scale"] == 0 and not r["stale"]
                )
            ),
            shield_samples=sum(r["shield"] for r in decisions),
            infeasible_samples=sum(
                not r["feasible"] and not r["stale"] for r in decisions
            ),
            decision_ms_p95=float(
                np.percentile([r["decision_ms"] for r in decisions], 95)
            ),
            plans=data["plans"],
            scope="tracked pedestrians; kinematic spline timing; no joint spatial optimizer",
        )
    except Exception as exc:
        outcome["error"] = repr(exc)
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        log.close()
        (args.output / "result.json").write_text(
            json.dumps(outcome, indent=2, allow_nan=False)
        )
        (args.output / "trace.json").write_text(
            json.dumps(
                dict(
                    poses=poses,
                    decisions=decisions,
                    plans=plans,
                    observations=observations,
                ),
                allow_nan=False,
            )
        )
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(outcome, indent=2), flush=True)
    return int(outcome["error"] is not None)


if __name__ == "__main__":
    raise SystemExit(main())
