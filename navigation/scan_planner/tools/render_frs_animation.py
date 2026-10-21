#!/usr/bin/env python3
"""Animate recorded sensor-driven FRS; no synthetic target predictions."""
import argparse, hashlib, json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from render_pointcloud_interaction import Render


def main():
    p = argparse.ArgumentParser()
    p.add_argument('run', type=Path)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--step', type=float, default=0.5)
    p.add_argument('--fps', type=float, default=4)
    args = p.parse_args()
    if args.step <= 0 or args.fps <= 0:
        p.error('step and fps must be positive')
    args.output.mkdir(parents=True, exist_ok=True)
    r = Render(args.run, Path(__file__).resolve().parents[4] / 'slash_robot_control')
    r.camera.distance = 7.15 * max(1.,r.goal[0]/3.)**.5
    font = lambda size: ImageFont.truetype(r.font, size)
    states = {'TRACKING': '正常通行', 'DYNAMIC_WAIT': '停车让行',
              'DYNAMIC_SLOW': '减速通行', 'GOAL_REACHED': '到达并停车',
              'IDLE': '等待启动', 'WAIT_MAP': '等待地图', 'WAIT_TRACK': '等待感知'}
    times = np.r_[np.arange(0, r.trace['poses'][-1]['t'], args.step), r.trace['poses'][-1]['t']]
    frames, records = [], []
    try:
        for i, t in enumerate(times):
            view, actual = r.image(float(t), i, '', clean=True)
            row = min(r.trace['poses'], key=lambda x: abs(x['t']-actual))
            tracks = max((x for x in r.trace['tracks'] if x['t'] <= actual),
                         key=lambda x: x['t'], default={})
            decision = max((x for x in r.trace['decisions'] if x['t'] <= actual),
                           key=lambda x: x['t'], default={})
            frame = Image.new('RGB', (1080, 800), 'white')
            frame.paste(view.resize((1080, 660), Image.Resampling.LANCZOS), (0, 65))
            d = ImageDraw.Draw(frame)
            d.text((26, 9), ('多动态障碍 FRS 感知与导航回放' if r.result.get('multi_specs') else '动态障碍 FRS 感知与导航回放'), font=font(27), fill='#243746')
            status = states.get(row['status'], row['status'] or '初始化')
            d.text((26, 45), f"t = {actual:4.1f} s   |   {status}   |   机器人速度 {row['v']:.2f} m/s", font=font(20), fill='#425463')
            objs = tracks.get('objects', [])
            text = '当前无活动目标' if tracks.get('healthy') else '感知等待 / 初始化'
            if objs:
                text = f'跟踪目标 {len(objs)} 个  |  ' + '   '.join(f"{o['id'].replace('obstacle_','T')} ({o['v'][0]:+.2f}, {o['v'][1]:+.2f})" for o in objs[:3]) + ' m/s'
            d.text((26, 704), text, font=font(19), fill='#425463')
            d.text((26, 735), '灰点：地图   蓝色：机器人轨迹   橙色：障碍物及位置 FRS（0.5 / 1 / 1.5 s）', font=font(19), fill='#425463')
            d.text((26, 768), f'实体为仿真真值回放，FRS 来自在线感知；软风险区域非硬禁入区。播放 {args.fps*args.step:g} 倍速', font=font(17), fill='#637681')
            frames.append(frame.quantize(colors=192))
            records.append({'time': actual, 'status': row['status'], 'frs_targets': len(decision.get('tracks', []))})
            if i % 10 == 0:
                print(f'rendered {i+1}/{len(times)}', flush=True)
        duration = [round(1000/args.fps)]*len(frames)
        duration[-1] = 1800
        frames[0].save(args.output/'frs_perception_navigation.gif', save_all=True,
                       append_images=frames[1:], duration=duration, loop=0, disposal=2, optimize=False)
        frames[min(range(len(times)), key=lambda i: abs(times[i]-6.5))].save(args.output/'preview.png')
        (args.output/'manifest.json').write_text(json.dumps({
            'source': str(args.run.resolve()),
            'trace_sha256': hashlib.sha256((args.run/'trace.json').read_bytes()).hexdigest(),
            'render_script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'frames': records, 'fps': args.fps, 'simulation_step': args.step,
            'note': 'Recorded software closed loop; ideal odometry and raycast depth. Truth entities replayed; online FRS unchanged. Position FRS slices only, without body inflation. No dynamics run by renderer.'
        }, ensure_ascii=False, indent=2)+'\n')
        print(args.output/'frs_perception_navigation.gif', flush=True)
    finally:
        r.renderer.close()


if __name__ == '__main__':
    main()
