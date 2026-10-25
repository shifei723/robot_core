#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真机录制数据丢帧拟合补全 + 可视化

背景
----
下位机控制周期 10 ms（100 Hz），上位机串口接收存在约 15~20% 的丢帧
（USB 接收缓冲溢出）。原始 `data.csv` 是不等间隔采样：相邻样本间隔在
8~12 ms 之间抖动，丢帧处出现 20 ms 以上的空档（最长约 100 ms）。

补全方法
--------
1. **细网格定位**：建 1 ms 等间隔网格，把每个原始样本映射到最近格点
   （相邻样本间隔 >= 8 ms，故不会碰撞，不丢任何样本）。
2. **缺口判定**：连续空档 >= 15 ms 判为丢帧缺口（正常 10 ms 抖动对应的
   空档约 8~12 ms，一次丢帧约 20 ms，阈值可干净区分）。
3. **局部多项式最小二乘拟合**：对每个缺口，取两侧各 0.35 s 内的**有效**
   样本，以窗口中值时刻为原点做 5 阶最小二乘拟合，再在缺口内取值
   （有效样本不足时自动降阶；仅 1 个有效点时常数填充）。
4. **常量列**（SBUS / 状态位 / 模式）取众数；`count` 取最近有效值
   （该计数器存在偶发回跳，不做外推）；`wall_time` 由 `t_s` 合成。
5. **输出**：抽取回 10 ms 等间隔（100 Hz，与控制周期一致）。

拟合精度验证
------------
随机遮挡 10% 的有效样本 → 用同一方法补全 → 与真值比较，逐信号给出
RMS / 最大误差及相对该信号标准差的百分比。这是缺口处拟合误差的无偏估计。

输出（不修改原始 data.csv）
--------------------------
    <目录>/data_filled.csv              补全后数据（列与原文件一致 + filled 标记列）
    <目录>/fig_gap_fill_<名>_full.pdf   全程对比图（原始点 / 拟合段 / 缺口底色）
    <目录>/fig_gap_fill_<名>_zoom.pdf   最大缺口局部放大对比图

用法
----
    python3 fill_gaps.py 录制数据/前向扰动 [录制数据/后向扰动 ...]
"""

import csv
import math
import sys
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np

FINE_DT = 0.001        # 细网格步长 / s
GAP_THRESH = 0.015     # 连续空档超过该时长判为丢帧缺口 / s
OUT_DT = 0.010         # 输出网格步长 / s（= 控制周期）
DEGREE = 5             # 局部多项式阶数
HALF_WIN = 0.35        # 拟合窗口半宽 / s
VAL_FRAC = 0.10        # 验证时随机遮挡的有效样本比例

CONST_COLS = [f"sbus_{i}" for i in range(10)] + ["status_code", "switched_on", "model"]
INT_COLS = ["count"] + CONST_COLS
TEXT_COLS = ["frame_type"]


# ------------------------------------------------------------------ 基础工具


def contiguous(mask):
    """mask 中 True 的连续段 [(i0, i1), ...]（左闭右开）"""
    segs, i, n = [], 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            segs.append((i, j))
            i = j
        else:
            i += 1
    return segs


def poly_fill(tg, y, degree=DEGREE, half_win=HALF_WIN):
    """局部多项式最小二乘补全 y 中的 NaN，返回新数组（不改原数组）"""
    out = np.array(y, dtype=float)
    miss = ~np.isfinite(out)
    valid = ~miss
    step = tg[1] - tg[0]
    w = max(degree + 2, int(round(half_win / step)))
    for i0, i1 in contiguous(miss):
        lo, hi = max(0, i0 - w), min(len(tg), i1 + w)
        idx = np.arange(lo, hi)
        v = valid[idx]
        nv = int(v.sum())
        if nv == 0:
            continue
        if nv == 1:                       # 只有一个有效点：常数填充
            out[i0:i1] = out[idx][v][0]
            continue
        d = min(degree, nv - 1)
        tc = tg[idx][v].mean()            # 居中以提高条件数
        try:
            coef = np.polyfit(tg[idx][v] - tc, out[idx][v], d)
        except np.linalg.LinAlgError:
            continue
        out[i0:i1] = np.polyval(coef, tg[i0:i1] - tc)
    return out


def validate(tg, y, gap_mask, seed=0):
    """随机遮挡有效样本后补全，返回 (误差, 信号峰峰值)"""
    ok = np.isfinite(y) & ~gap_mask
    cand = np.flatnonzero(ok)
    if len(cand) < 30:
        return None
    rng = np.random.default_rng(seed)
    hide = rng.choice(cand, size=max(1, int(len(cand) * VAL_FRAC)), replace=False)
    masked = np.array(y, dtype=float)
    masked[hide] = np.nan
    filled = poly_fill(tg, masked)
    err = filled[hide] - np.asarray(y, dtype=float)[hide]
    v = np.asarray(y, dtype=float)[cand]
    return err, float(v.max() - v.min())


# ------------------------------------------------------------------ 主流程


def process(d):
    d = Path(d)
    with open(d / "data.csv", newline="") as fh:
        rows = list(csv.DictReader(fh))
    fields = list(rows[0].keys())
    t = np.array([float(r["t_s"]) for r in rows])

    # ---- 细网格 + 样本定位 ----
    t0, tend = t[0], t[-1]
    tg = np.arange(t0, tend + 1e-9, FINE_DT)
    nf = len(tg)
    src = np.full(nf, -1, dtype=int)
    for k, x in enumerate(t):
        i = int(np.clip(round((x - t0) / FINE_DT), 0, nf - 1))
        src[i] = k                        # 相邻样本 >= 8 ms，不会碰撞
    have = src >= 0

    # ---- 缺口判定：连续空档 >= GAP_THRESH ----
    gap_mask = np.zeros(nf, dtype=bool)
    for a, b in contiguous(~have):
        if (tg[min(b, nf - 1)] - tg[a]) >= GAP_THRESH - 1e-9:
            gap_mask[a:b] = True
    gaps = contiguous(gap_mask)
    n_gap_ms = int(gap_mask.sum())
    n_miss = int(round(n_gap_ms / (OUT_DT / FINE_DT)))

    print(f"\n=== {d.name} ===")
    print(f"  原始样本 {len(t)}  细网格 {nf} 点（1 ms）")
    print(f"  丢帧缺口 {len(gaps)} 段，合计 {n_gap_ms} ms ≈ {n_miss} 帧"
          f"（占标称 100 Hz 的 {n_miss / (n_miss + len(t)) * 100:.1f}%）")
    if gaps:
        lens = [(tg[min(b, nf - 1)] - tg[a]) * 1000 for a, b in gaps]
        print(f"  缺口时长: 最短 {min(lens):.0f} ms  最长 {max(lens):.0f} ms  "
              f"平均 {np.mean(lens):.1f} ms")

    # ---- 逐列补全（在细网格上进行）----
    fine = {c: np.full(nf, np.nan) for c in fields}
    fine["t_s"] = tg
    base_wall = datetime.fromisoformat(rows[0]["wall_time"])
    report = []

    for c in fields:
        if c in ("t_s", "wall_time"):
            continue
        if c in TEXT_COLS:
            seq = [rows[src[i]][c] if src[i] >= 0 else "" for i in range(nf)]
            last = next((x for x in seq if x), "")       # 文本列：取最近有效值
            filled_seq = []
            for x in seq:
                if x:
                    last = x
                filled_seq.append(last)
            fine[c] = np.array(filled_seq, dtype=object)
            continue
        v = np.array([float(r[c]) for r in rows])
        g = np.full(nf, np.nan)
        g[have] = v[src[have]]
        if c in CONST_COLS:
            vals, cnts = np.unique(v, return_counts=True)
            g = np.where(np.isfinite(g), g, vals[np.argmax(cnts)])
        else:
            if c != "count":
                res = validate(tg, g, gap_mask)
                if res is not None:
                    err, pp = res
                    report.append((c, float(np.sqrt(np.mean(err ** 2))),
                                   float(np.max(np.abs(err))), pp))
            g = poly_fill(tg, g)
        fine[c] = g

    # ---- 抽取回 10 ms ----
    step = int(round(OUT_DT / FINE_DT))
    out_idx = np.arange(0, nf, step)
    out_t = tg[out_idx]
    out_flag = gap_mask[out_idx]
    n_out = len(out_idx)
    out = {c: (fine[c][out_idx] if c not in ("t_s", "wall_time") else None)
           for c in fields}

    def col(c):
        if c == "t_s":
            return out_t
        if c == "wall_time":
            return [(base_wall + timedelta(seconds=float(x))).isoformat(timespec="milliseconds")
                    for x in out_t]
        return fine[c][out_idx]

    # ---- 写出 ----
    dst = d / "data_filled.csv"
    with open(dst, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(fields + ["filled"])
        for i in range(n_out):
            row = []
            for c in fields:
                val = col(c)[i]
                if c in TEXT_COLS or c == "wall_time":
                    row.append(val)
                elif c in INT_COLS:
                    row.append(str(int(round(float(val)))))
                else:
                    row.append(repr(float(val)))
            row.append("1" if out_flag[i] else "0")
            w.writerow(row)
    n_filled = int(out_flag.sum())
    print(f"  已写出 {dst}（{n_out} 行 @100 Hz，其中拟合补全 {n_filled} 行，"
          f"{n_filled / n_out * 100:.1f}%）")

    # ---- 拟合精度报告 ----
    # 说明: 陀螺/加速度等信号以噪声为主, 拟合值相当于去噪后的趋势,
    #       其"误差"接近噪声幅度是必然的; 判断能否用要看相对信号变化范围的比例。
    KEY = ["pitch_deg", "speed", "lw_speed", "rw_speed", "roll_deg", "yaw_deg",
           "m4_torque", "m5_torque", "m0_pos", "m1_pos", "m2_pos", "m3_pos"]
    print("  拟合精度（随机遮挡 10% 有效样本后补全，与真值比较）：")
    print(f"    {'信号':<14}{'RMS':>10}{'最大':>10}{'峰峰值':>10}{'RMS/pp':>9}")
    byname = {c: (r, m, p) for c, r, m, p in report}
    for c in KEY:
        if c in byname:
            r, m, p = byname[c]
            print(f"    {c:<14}{r:>10.4f}{m:>10.4f}{p:>10.4f}"
                  f"{r / p * 100 if p else 0:>8.2f}%")
    rest = [(c, r, m, p) for c, r, m, p in report if c not in KEY]
    rest.sort(key=lambda x: -(x[1] / x[3] if x[3] else 0))
    print(f"    其余信号中相对误差最大的 {min(8, len(rest))} 个：")
    for c, r, m, p in rest[:8]:
        print(f"    {c:<14}{r:>10.4f}{m:>10.4f}{p:>10.4f}"
              f"{r / p * 100 if p else 0:>8.2f}%")

    return dict(tg=out_t, tg_fine=tg, out=out, flag=out_flag, gaps=gaps,
                gap_fine=gap_mask, fields=fields, name=d.name)


# ------------------------------------------------------------------ 可视化

BLUE, ORANGE, GRAY = "#28618d", "#be6b36", "#79828a"


def setup_mpl():
    import matplotlib
    import matplotlib.texmanager as _tm
    _tm._usepackage_if_not_loaded = lambda *a, **k: ""   # 跳过未安装的 underscore.sty
    matplotlib.use("pgf")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "pgf.texsystem": "xelatex", "pgf.rcfonts": False,
        "pgf.preamble": r"""\usepackage{mathspec}\usepackage{xeCJK}
\IfFontExistsTF{Songti SC}{\setCJKmainfont[AutoFakeBold=true]{Songti SC}}{\setCJKmainfont[AutoFakeBold=true]{FandolSong-Regular.otf}}
\IfFontExistsTF{Times New Roman}{\setallmainfonts{Times New Roman}}{\setallmainfonts{TeX Gyre Termes}}""",
        "font.family": "serif", "font.size": 10.5, "axes.unicode_minus": False,
        "axes.linewidth": .6, "xtick.major.width": .6, "ytick.major.width": .6,
        "legend.frameon": False, "legend.fontsize": 9,
    })
    return plt


def draw_series(R):
    """返回 [(轴标签, 补全后序列), ...]（10 ms 输出网格）"""
    out, tg = R["out"], R["tg"]
    hub = 0.5 * (out["m4_torque"] + out["m5_torque"])
    sp = out["speed"]
    disp = np.zeros(len(tg))
    disp[1:] = np.cumsum(0.5 * (sp[1:] + sp[:-1]) * (tg[1:] - tg[:-1]))
    disp -= disp[0]
    return [("俯仰角 / °", out["pitch_deg"]),
            ("轮毂力矩 / N·m", hub),
            ("轮速 / (m/s)", sp),
            ("纵向位移 / m", disp)]


def main():
    plt = setup_mpl()
    for d in sys.argv[1:]:
        R = process(d)
        d = Path(d)
        tg, flag = R["tg"], R["flag"]
        have = flag == 0
        series = draw_series(R)
        T = int(math.ceil(tg[-1]))
        gf = R["gap_fine"]
        tgf = R["tg_fine"]

        # ---------- 全程对比 ----------
        fig, axs = plt.subplots(4, 1, figsize=(6.6, 7.6))
        fig.subplots_adjust(left=.13, right=.97, bottom=.07, top=.93, hspace=.42)
        for ax, (ylab, y) in zip(axs, series):
            ax.spines[["top", "right"]].set_visible(False)
            ax.spines[["left", "bottom"]].set_color("#a6abb0")
            ax.grid(axis="y", color="#e5e8eb", lw=.5)
            ax.set_axisbelow(True)
            ax.tick_params(labelsize=9, length=3)
            ax.set_xlabel("时间 / s", labelpad=2)
            ax.set_ylabel(ylab, labelpad=2)
            for a, b in R["gaps"]:
                ax.axvspan(tgf[a], tgf[min(b, len(tgf) - 1)], color=ORANGE,
                           alpha=.13, lw=0)
            ax.plot(tg[have], y[have], "o", ms=1.6, color=BLUE)
            ax.plot(tg[~have], y[~have], color=ORANGE, lw=1.3)
            ax.set_xlim(0, T)
            ax.set_xticks(range(0, T + 1, 2))
            ax.margins(y=.2)
        axs[0].set_title("圆点=原始有效样本    橙线=拟合补全    底色=丢帧缺口",
                         fontsize=9, color=GRAY, pad=6)
        p1 = d / f"fig_gap_fill_{R['name']}_full.pdf"
        fig.savefig(p1)
        plt.close(fig)

        # ---------- 最大缺口放大 ----------
        if R["gaps"]:
            a, b = max(R["gaps"], key=lambda g: g[1] - g[0])
            c, e = max(a - 400, 0), min(b + 400, len(tgf))
            tw0, tw1 = tgf[c], tgf[min(e, len(tgf) - 1)]
            sel = (tg >= tw0) & (tg <= tw1)
            fig, axs = plt.subplots(4, 1, figsize=(6.6, 7.6))
            fig.subplots_adjust(left=.13, right=.97, bottom=.07, top=.93, hspace=.42)
            for ax, (ylab, y) in zip(axs, series):
                ax.spines[["top", "right"]].set_visible(False)
                ax.spines[["left", "bottom"]].set_color("#a6abb0")
                ax.grid(axis="y", color="#e5e8eb", lw=.5)
                ax.set_axisbelow(True)
                ax.tick_params(labelsize=9, length=3)
                ax.set_xlabel("时间 / s", labelpad=2)
                ax.set_ylabel(ylab, labelpad=2)
                ax.axvspan(tgf[a], tgf[min(b, len(tgf) - 1)], color=ORANGE,
                           alpha=.13, lw=0)
                tt, yy, ff = tg[sel], y[sel], flag[sel]
                ax.plot(tt[~ff], yy[~ff], "o", ms=3.5, color=BLUE)
                ax.plot(tt[ff], yy[ff], color=ORANGE, lw=1.6)
                ax.plot(tt[~ff], yy[~ff], color=BLUE, lw=.7, alpha=.55)
                ax.set_xlim(tt[0], tt[-1])
            axs[0].set_title(
                f"最大缺口放大 {tgf[a]:.2f}–{tgf[b-1]:.2f}s"
                f"（缺失 {(tgf[b-1] - tgf[a]) * 1000:.0f} ms）"
                "    圆点=原始样本    橙线=拟合补全", fontsize=9, color=GRAY, pad=6)
            p2 = d / f"fig_gap_fill_{R['name']}_zoom.pdf"
            fig.savefig(p2)
            plt.close(fig)
            print(f"  已写出 {p1} 与 {p2}")


if __name__ == "__main__":
    main()
