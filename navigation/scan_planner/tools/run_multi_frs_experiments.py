#!/usr/bin/env python3
"""Run the real ROS spline controller, not the legacy Python MPC benchmark.

Source robot_core/install/ws_nav_frs/setup.bash first; use system Python.
Three isolated ROS domains execute one prediction method each.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--robot-core", type=Path, default=Path(__file__).resolve().parents[3]
    )
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--seeds", type=int, nargs="+", default=[100, 101, 102])
    p.add_argument(
        "--cases",
        nargs="+",
        choices=["crossing", "combined"],
        default=["crossing", "combined"],
    )
    p.add_argument("--domain", type=int, default=151)
    p.add_argument("--pedestrians", type=int, choices=[1, 3], default=3)
    p.add_argument("--timeout", type=float, default=100)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    root = args.robot_core.resolve()
    tools = root / "navigation/scan_planner/tools"
    sources = [
        tools / "validate_dynamic_navigation.py",
        tools / "validate_height_navigation.py",
        root
        / "navigation/scan_planner/src/planner/plan_manage/include/plan_manage/intent_frs.hpp",
        root
        / "navigation/scan_planner/src/planner/plan_manage/src/closed_loop_controller.cpp",
        root / "navigation/scan_planner/src/planner/plan_manage/launch/run.launch.py",
    ]
    snapshot = args.output / "source_snapshot"
    snapshot.mkdir()
    hashes = {}
    for src in sources:
        blob = src.read_bytes()
        (snapshot / src.name).write_bytes(blob)
        hashes[str(src.relative_to(root))] = hashlib.sha256(blob).hexdigest()
    protocol = dict(
        seeds=args.seeds,
        cases=args.cases,
        modes=["cv", "frs", "physical"],
        source_sha256=hashes,
        ros_distro=os.environ.get("ROS_DISTRO"),
        scope="kinematic B-spline timing with multiple ideal tracked pedestrians; no spatial joint optimization",
        pedestrians=args.pedestrians,
        timeout_s=args.timeout,
        sampling="10 Hz, fixed time indices; paired independent noise per index",
        noise=dict(position_std_m=0.002, velocity_std_m_s=0.01),
        dt_prediction_s=0.1,
        horizon_s=3.0,
        robot_disc_radius_m=0.43,
        pedestrian_radius_m=0.30,
        margin_m=0.10,
        development_seeds=[0],
        test_seeds=args.seeds,
    )
    (args.output / "protocol.json").write_text(json.dumps(protocol, indent=2))

    def worker(item):
        offset, mode = item
        results = []
        for case in args.cases:
            for seed in args.seeds:
                output = args.output / f"{case}_{mode}_{seed}"
                with (args.output / f"{case}_{mode}_{seed}.log").open("w") as log:
                    code = subprocess.call(
                        [
                            sys.executable,
                            str(tools / "validate_dynamic_navigation.py"),
                            "--mode",
                            mode,
                            "--pedestrians",
                            str(args.pedestrians),
                            "--timeout",
                            str(args.timeout),
                            "--case",
                            case,
                            "--seed",
                            str(seed),
                            "--domain",
                            str(args.domain + offset),
                            "--output",
                            str(output),
                        ],
                        stdout=log,
                        stderr=subprocess.STDOUT,
                    )
                if (output / "result.json").is_file():
                    result = json.loads((output / "result.json").read_text())
                else:
                    result = dict(
                        mode=mode,
                        case=case,
                        seed=seed,
                        error="runner failed before result",
                        reached=False,
                    )
                result["exit_code"] = code
                results.append(result)
                print(
                    f"{case} {mode} seed={seed}: reached={result['reached']} error={result['error']}",
                    flush=True,
                )
        return results

    with ThreadPoolExecutor(max_workers=3) as executor:
        groups = list(executor.map(worker, enumerate(protocol["modes"])))
    results = [r for group in groups for r in group]
    (args.output / "results.json").write_text(
        json.dumps(results, indent=2, allow_nan=False)
    )
    return int(any(r["error"] is not None for r in results))


if __name__ == "__main__":
    raise SystemExit(main())
