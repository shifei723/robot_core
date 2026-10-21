"""Complete paired-run comparison and fixed-seed multi-target diagnostics."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    args = parser.parse_args()
    root = args.results
    rows = json.loads((root / "results.json").read_text())
    protocol = json.loads((root / "protocol.json").read_text())
    cases = protocol["cases"]
    modes = ["cv", "frs", "physical"]
    colors = ["#4477aa", "#228833", "#cc6677"]
    summary = []
    fig, axes = plt.subplots(
        len(cases),
        3,
        figsize=(12, 3.6 * len(cases)),
        layout="constrained",
        squeeze=False,
    )
    for ci, case in enumerate(cases):
        for mi, mode in enumerate(modes):
            all_runs = [r for r in rows if r["case"] == case and r["mode"] == mode]
            valid = [r for r in all_runs if r["error"] is None]
            r = dict(
                case=case,
                method=mode,
                total=len(all_runs),
                errors=len(all_runs) - len(valid),
                reached=sum(x["reached"] for x in valid),
                collisions=sum(x["collision"] for x in valid),
                shield_samples=sum(x["shield_samples"] for x in valid),
            )
            for j, key in enumerate(["duration_s", "min_clearance_m", "wait_s"]):
                values = [x[key] for x in valid]
                if values:
                    r[key] = float(np.mean(values))
                    r[key + "_std"] = (
                        float(np.std(values, ddof=1)) if len(values) > 1 else 0
                    )
                    axes[ci, j].bar(mi, r[key], color=colors[mi], alpha=0.6)
                    axes[ci, j].scatter([mi] * len(values), values, color=colors[mi])
                axes[ci, j].set(
                    title=f"{case}: {key}", xticks=range(3), xticklabels=modes
                )
            summary.append(r)
    fig.suptitle("Three dynamic obstacles; all runs retained; dots = independent seeds")
    for suffix in ["png", "pdf"]:
        fig.savefig(root / f"comparison.{suffix}", dpi=200)
    (root / "summary.json").write_text(json.dumps(summary, indent=2))
    with (root / "comparison.csv").open("w") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)
    seed = protocol["seeds"][0]
    for case in cases:
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), layout="constrained")
        for mode, color in zip(modes, colors):
            trace = json.loads(
                (root / f"{case}_{mode}_{seed}" / "trace.json").read_text()
            )
            if not trace["decisions"]:
                continue
            d = trace["decisions"]
            t = [x["t"] for x in d]
            axes[0].plot(t, [x["v"] for x in d], color=color, label=mode)
            axes[1].step(t, [x["scale"] for x in d], where="post", color=color)
            poses = trace["poses"]
            axes[2].plot(
                [x["t"] for x in poses], [x["pose"][2] for x in poses], color=color
            )
        axes[0].set_ylabel("Command speed (m/s)")
        axes[0].legend()
        axes[1].set_ylabel("Progression scale")
        axes[2].set(xlabel="Time (s)", ylabel="Base height (m)")
        for suffix in ["png", "pdf"]:
            fig.savefig(root / f"{case}_response.{suffix}", dpi=200)
        trace = json.loads((root / f"{case}_frs_{seed}" / "trace.json").read_text())
        if not trace["decisions"]:
            continue
        fig, axes = plt.subplots(2, 2, figsize=(10, 9), layout="constrained")
        target_colors = ["#cc3311", "#8844aa", "#dd9911"]
        for ax, t in zip(axes.flat, [4, 16, 28, 40]):
            row = min(trace["decisions"], key=lambda x: abs(x["t"] - t))
            for target in row["tracks"]:
                identity = int(target["id"].split("_")[-1])
                color = target_colors[identity]
                for k, alpha in [(9, 0.3), (19, 0.12)]:
                    _, x, y, hx, hy = target["boxes"][k]
                    ax.add_patch(
                        Rectangle(
                            (x - hx, y - hy), 2 * hx, 2 * hy, color=color, alpha=alpha
                        )
                    )
                obs = [o for o in trace["observations"] if o["id"] == target["id"]]
                current = min(obs, key=lambda o: abs(o["t"] - row["t"]))
                ax.scatter(*current["position"], color=color, label=target["id"])
            ax.add_patch(
                Circle((row["x"], row["y"]), 0.43, color="#4477aa", fill=False)
            )
            past = np.array([p["pose"][:2] for p in trace["poses"] if p["t"] <= t])
            if len(past):
                ax.plot(past[:, 0], past[:, 1], color="#4477aa")
            if case == "combined":
                ax.add_patch(Rectangle((-1, -0.3), 2, 0.6, color="gray", alpha=0.15))
            ax.plot(0, 2, "g*")
            ax.set(
                xlim=(-3, 3),
                ylim=(-2.6, 2.6),
                aspect="equal",
                title=f"t={row['t']:.1f}s, scale={row['scale']:.1f}",
            )
        axes[0, 0].legend(fontsize=8)
        fig.suptitle(f"{case}: fixed seed {seed}; raw 1 s / 2 s FRS; local view")
        for suffix in ["png", "pdf"]:
            fig.savefig(root / f"{case}_frs_snapshots.{suffix}", dpi=200)
    report = [
        "# Multi-obstacle ROS comparison",
        "",
        "All methods share the same geometry planner, controller, candidate set and safety layer.",
        "Three ideal tracked pedestrians; kinematic simulation, not contact dynamics.",
        "",
        "| Scene | Method | Reached | Collisions | Time (s) | Min gap (m) | Pause (s) |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for r in summary:
        report.append(
            f"| {r['case']} | {r['method']} | {r['reached']}/{r['total']} | {r['collisions']} | {r.get('duration_s', float('nan')):.2f} | {r.get('min_clearance_m', float('nan')):.3f} | {r.get('wait_s', float('nan')):.2f} |"
        )
    report += [
        "",
        "Pause counts zero progression and can overlap height-control holds. All failures and timeouts are retained.",
        "Source states and predictions are in each trace.json. The first evaluation seed is fixed for figures, not selected by performance.",
    ]
    (root / "REPORT.md").write_text("\n".join(report))


if __name__ == "__main__":
    main()
