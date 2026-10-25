#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真机录制数据剪辑工具 —— 剔除指定的操作段, 保留其余部分

典型用法: 一次跳跃/扰动实验的录制里混入了「转向调整」, 需要把这些段
剪掉, 剩下的才是干净的实验数据。

    python3 clip_recording.py 录制数据/跳跃1                 # 自动检测并预览
    python3 clip_recording.py 录制数据/跳跃1 --apply          # 实际生成
    python3 clip_recording.py 录制数据/跳跃1 --cut 6.5,7.3 --cut 27.8,28.5 --apply

设计要点
--------
1. 定界依据优先用「操作输入」(SBUS 转向通道偏离中位), 而不是姿态响应:
   遥令先于响应约 0.1 s, 且能避免把「站立微摆」误判成转向。
   三重确认: 摇杆偏离 + 持续时长 + 该段 yaw 确实转了 >2°。
2. 段尾延长到「角速度回落并保持」, 把转完之后的回稳尾巴一并剪掉,
   避免拼接处残留未收敛的角速度。
3. 输出的 t_s 是「扣掉被剪段落后的连续时间」(从 0 起), 并额外给出
   t_raw(原始录制时间) 与 seg(所属片段编号), 便于回溯与分段绘图。
"""

import argparse
import csv
import json
import os
import sys

DEFAULT_TURN_CHANNEL = "sbus_3"


def load_csv(path):
    with open(path, encoding="utf-8") as fh:
        rd = csv.DictReader(fh)
        fields = rd.fieldnames
        rows = list(rd)
    return fields, rows


def detect_turns(rows, chan=DEFAULT_TURN_CHANNEL, dev=30, min_sec=0.2,
                 min_dyaw=2.0, tail_sec=1.5, av_key="angular_vel",
                 av_quiet=0.05):
    """自动检测「转向调整」段, 返回 [(idx_a, idx_b_exclusive), ...]"""
    if chan not in rows[0]:
        return []
    # 中位 = 出现最多的值
    vals = [float(r[chan]) for r in rows]
    mid = max(set(vals), key=vals.count)
    active = [abs(v - mid) >= dev for v in vals]
    # 合并短暂回中的间隙 (<0.3 s)
    gap_max = int(0.3 / 0.01)
    i = 0
    while i < len(active):
        if not active[i]:
            j = i
            while j < len(active) and not active[j]:
                j += 1
            if 0 < j - i <= gap_max and i > 0 and j < len(active):
                for k in range(i, j):
                    active[k] = True
            i = j
        else:
            i += 1
    t = [float(r["t_s"]) for r in rows]
    av = [abs(float(r[av_key])) for r in rows]
    yaw = [float(r["yaw_deg"]) for r in rows]

    def dyaw(a, b):                       # a..b 的 yaw 净变化 (处理 ±180 缠绕)
        s = 0.0
        for k in range(a, b):
            d = yaw[k + 1] - yaw[k]
            if d > 180: d -= 360
            if d < -180: d += 360
            s += d
        return s

    segs = []
    i = 0
    while i < len(active):
        if active[i]:
            j = i
            while j < len(active) and active[j]:
                j += 1
            a, b = i, j                                  # 半开区间 [a, b)
            # 尾部延长到角速度安静下来 (最多 tail_sec)
            k = b
            lim = min(len(rows) - 1, b + int(tail_sec / 0.01))
            quiet = 0
            while k < lim:
                if av[k] < av_quiet:
                    quiet += 1
                    if quiet >= 10:                      # 连续 10 帧安静即收敛
                        break
                else:
                    quiet = 0
                k += 1
            b = k - quiet + 1 if quiet >= 10 else b
            if t[b - 1] - t[a] >= min_sec and abs(dyaw(a, b)) >= min_dyaw:
                segs.append((a, b))
            i = j
        else:
            i += 1
    return segs


def parse_cuts(pairs):
    out = []
    for s in pairs:
        a, b = s.split(",")
        out.append((float(a), float(b)))
    return out


def build_plan(rows, cuts):
    """把删除区间转成保留区间 [[a,b), ...]"""
    t = [float(r["t_s"]) for r in rows]
    keep, prev = [], 0.0
    for a, b in sorted(cuts):
        if a > prev:
            keep.append((prev, a))
        prev = max(prev, b)
    if prev < t[-1]:
        keep.append((prev, t[-1] + 1e-9))
    return keep


def main():
    ap = argparse.ArgumentParser(description="剪辑真机录制数据")
    ap.add_argument("src", help="录制数据目录 (含 data.csv / meta.json)")
    ap.add_argument("--out", default=None, help="输出目录, 默认 <src>_clip")
    ap.add_argument("--turn-channel", default=DEFAULT_TURN_CHANNEL)
    ap.add_argument("--cut", action="append", default=[],
                    help="手动指定删除区间 '起,止' (秒), 可重复; 指定后跳过自动检测")
    ap.add_argument("--dev", type=float, default=30,
                    help="转向摇杆偏离中位多少 LSB 算有效输入 (默认 30)")
    ap.add_argument("--apply", action="store_true", help="实际写出, 否则仅预览")
    args = ap.parse_args()

    src = args.src.rstrip("/")
    csv_path = os.path.join(src, "data.csv")
    meta_path = os.path.join(src, "meta.json")
    if not os.path.exists(csv_path):
        sys.exit("找不到 %s" % csv_path)
    fields, rows = load_csv(csv_path)
    t = [float(r["t_s"]) for r in rows]

    if args.cut:
        cuts_t = parse_cuts(args.cut)
        cuts = [(a, b) for a, b in cuts_t]
        origin = "manual"
    else:
        segs = detect_turns(rows, args.turn_channel, dev=args.dev)
        cuts = [(t[a], t[b - 1]) for a, b in segs]
        origin = "auto"

    print("源文件: %s  (%d 帧, %.2f s)" % (csv_path, len(rows), t[-1] - t[0]))
    print("定界方式: %s" % ("自动检测 (%s)" % args.turn_channel if origin == "auto"
                            else "手动指定"))
    print("\n将被删除的段:")
    drops = 0
    for a, b in cuts:
        ia = next(i for i, x in enumerate(t) if x >= a)
        ib = next((i for i, x in enumerate(t) if x > b), len(t))
        dyaw = None
        print("  %6.3f → %6.3f   (%5.3f s, %d 帧)"
              % (a, b, b - a, ib - ia))
        drops += ib - ia
    print("  合计删除 %d 帧 (%.3f s)" % (drops, sum(b - a for a, b in cuts)))

    keep = build_plan(rows, cuts)
    print("\n保留段:")
    nk = 0
    for a, b in keep:
        ia = next(i for i, x in enumerate(t) if x >= a)
        ib = next((i for i, x in enumerate(t) if x >= b), len(t))
        print("  段%d  %6.3f → %6.3f  (%5.3f s, %d 帧)"
              % (len([k for k in keep[:keep.index((a, b))]]) + 1,
                 a, min(b, t[-1]), t[ib - 1] - t[ia], ib - ia))
        nk += ib - ia
    print("  合计保留 %d 帧" % nk)

    if not args.apply:
        print("\n[预览模式] 加 --apply 才会写出文件")
        return

    out = args.out or (src + "_clip")
    os.makedirs(out, exist_ok=True)
    out_csv = os.path.join(out, "data.csv")

    new_fields = ["seg", "t_s", "t_raw"] + \
                 [f for f in fields if f not in ("seg", "t_s", "t_raw")]
    t0 = t[0]
    with open(out_csv, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=new_fields)
        w.writeheader()
        seg_no, carry = 1, 0.0
        prev_cut_end = None
        for si, (a, b) in enumerate(keep):
            ia = next(i for i, x in enumerate(t) if x >= a)
            ib = next((i for i, x in enumerate(t) if x >= b), len(t))
            if si > 0:                       # 扣除被剪掉的时间, 保持连续
                carry += a - keep[si - 1][1]
            for i in range(ia, ib):
                r = dict(rows[i])
                raw = r["t_s"]
                r["seg"] = seg_no
                r["t_raw"] = raw
                r["t_s"] = "%.6f" % (float(raw) - t0 - carry)
                w.writerow(r)
            seg_no += 1

    meta = {}
    if os.path.exists(meta_path):
        with open(meta_path, encoding="utf-8") as fh:
            meta = json.load(fh)
    nj = len([r for r in open(out_csv, encoding="utf-8")]) - 1
    new_meta = dict(meta)
    # 直接用时间算跨度, 别拿时间值当列表索引
    dur_kept = sum(min(b, t[-1]) - a for a, b in keep)
    new_meta.update({
        "name": (meta.get("name", src) or src) + " (剪辑后)",
        "source_dir": os.path.abspath(src),
        "clip_origin": origin,
        "clip_cut_ranges_s": [[round(a, 3), round(b, 3)] for a, b in cuts],
        "clip_keep_ranges_s": [[round(a, 3), round(b, 3)] for a, b in keep],
        "frames_clipped": nj,
        "duration_clipped_s": round(dur_kept, 3),
        "cut_seconds": round(sum(b - a for a, b in cuts), 3),
    })
    with open(os.path.join(out, "meta.json"), "w", encoding="utf-8") as fh:
        json.dump(new_meta, fh, ensure_ascii=False, indent=2)

    print("\n已写出: %s (%d 帧)" % (out_csv, nj))
    print("        %s" % os.path.join(out, "meta.json"))


if __name__ == "__main__":
    main()
