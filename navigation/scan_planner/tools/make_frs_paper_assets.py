"""Assemble fixed-time, fixed-seed EGL frames into publication figures."""

import argparse
import json
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    args = parser.parse_args()
    root = args.results
    protocol = json.loads((root / "protocol.json").read_text())
    seed = protocol["seeds"][0]
    output = root / "paper_assets"
    output.mkdir(exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(12.8, 8.4), layout="constrained")
    for label, t, ax in zip("abcd", [4, 16, 28, 40], axes.flat):
        ax.imshow(plt.imread(root / f"headless_frs_{seed}" / f"frame_{t}s.png"))
        ax.axis("off")
        ax.set_title(f"({label}) t = {t} s", fontsize=12)
    for extension in ["png", "pdf"]:
        fig.savefig(output / f"multi_frs_headless.{extension}", dpi=300)
    fig, axes = plt.subplots(1, 3, figsize=(16, 4), layout="constrained")
    for mode, ax in zip(["cv", "frs", "physical"], axes):
        ax.imshow(plt.imread(root / f"headless_{mode}_{seed}" / "frame_16s.png"))
        ax.axis("off")
        ax.set_title(mode.upper(), fontsize=12)
    for extension in ["png", "pdf"]:
        fig.savefig(output / f"multi_method_headless.{extension}", dpi=300)
    caption = (
        "三动态障碍与低横梁场景的无头三维回放。固定评估种子为 "
        f"{seed}，展示第 4、16、28、40 秒；机器人位姿、高度和 FRS 来自实际 ROS 运行日志。"
        "红、紫、橙色分别表示三个独立跟踪目标，同色地面矩形为未来 1 秒和 2 秒的原始位置可达集，"
        "蓝色点线表示机器人已执行轨迹，绿色圆盘为导航目标，地面网格间距为 1 米。"
        "画面由 MuJoCo EGL 渲染 Slash 模型与脚本化场景，属于运动学日志重建，未执行接触动力学仿真。"
        "预测区域不含机器人包络膨胀；镜头为局部裁剪，目标离开视野后仍保留其原始轨迹记录。"
    )
    (output / "caption_zh.txt").write_text(caption)

    summary = json.loads((root / "summary.json").read_text())
    lines = [
        r"\begin{table}[htbp]",
        r"\centering\small",
        r"\caption{三动态障碍对照结果（各组 3 次，均值 $\pm$ 样本标准差）}",
        r"\label{tab:ch5_multi_frs}",
        r"\begin{tabular}{llcccc}\toprule",
        r"场景 & 方法 & 到达/总数 & 用时（s） & 最小间距（m） & 暂停（s） \\",
        r"\midrule",
    ]
    names = {"cv": "CV", "frs": "Intent-FRS", "physical": "Physical-FRS"}
    for row in summary:
        scene = "普通横穿" if row["case"] == "crossing" else "低横梁组合"
        values = []
        for key, digits in [("duration_s", 2), ("min_clearance_m", 3), ("wait_s", 2)]:
            values.append(f"${row[key]:.{digits}f} \\pm {row[key + '_std']:.{digits}f}$")
        lines.append(
            f"{scene} & {names[row['method']]} & {row['reached']}/{row['total']} & "
            + " & ".join(values) + r" \\"
        )
    lines.extend([r"\bottomrule\end{tabular}", r"\end{table}"])
    (output / "comparison_table.tex").write_text("\n".join(lines) + "\n")



if __name__ == "__main__":
    main()
