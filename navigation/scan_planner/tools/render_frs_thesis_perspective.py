#!/usr/bin/env python3
"""Three consistent oblique replay views, matching the original process_2 style."""
import argparse
import hashlib
import json
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
from render_pointcloud_interaction import Render


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    r = Render(a.run, Path(__file__).resolve().parents[4] / 'slash_robot_control')
    # Same azimuth/elevation as process_2; modestly widen the frame to retain walls/FRS.
    r.camera.distance = 7.15
    r.camera.lookat[:] = [1.5, 0, .15]
    titles = ['感知建图与动态目标识别', 'FRS 引导下的交互避障', '恢复通行并到达目标']
    times = [4., 9., r.trace['poses'][-1]['t']]
    panels = []
    for i, (t, title) in enumerate(zip(times, titles), 1):
        img, actual = r.image(t, i, title)
        d = ImageDraw.Draw(img)
        d.rectangle([0, 0, 1800, 109], fill='white')
        row = min(r.trace['poses'], key=lambda x: abs(x['t']-actual))
        font = lambda n: ImageFont.truetype(r.font, n)
        d.text((48, 16), f'({chr(96+i)}) {title}', font=font(36), fill='#18344a')
        state = ['动态感知', '停车让行', '到达停车'][i-1]
        d.text((48, 66), f't = {actual:.2f} s     v = {row["v"]:.2f} m/s     {state}', font=font(26), fill='#526777')
        # Keep exact colour/style conventions of the requested reference.
        d.rectangle([0, 1000, 1800, 1100], fill='white')
        for x, col, txt in [(48, '#275f9b', 'Slash 与历史运动拖影'), (580, '#b74e23', '障碍物与历史运动拖影'), (1140, '#ce8b24', '位置 FRS：0.5 / 1 / 1.5 s')]:
            d.rectangle([x, 1023, x+24, 1047], fill=col)
            d.text((x+35, 1014), txt, font=font(25), fill='#263f51')
        d.text((48, 1060), '灰点：静态底图与实时观测；绿圈：目标；红虚线：当前组合半径。实体为真值回放，FRS 为在线估计。', font=font(22), fill='#637681')
        stem = a.output / f'process_{i}'
        img.save(stem.with_suffix('.png'), dpi=(300, 300))
        img.save(stem.with_suffix('.pdf'), resolution=300)
        panels.append({'file':stem.name, 't':actual, 'speed':row['v'], 'status':row['status']})
    r.renderer.close()
    manifest = {'source':str(a.run.resolve()), 'trace_sha256':hashlib.sha256((a.run/'trace.json').read_bytes()).hexdigest(), 'camera':{'azimuth':140, 'elevation':-52, 'distance':7.15, 'lookat':[1.5,0,.15]}, 'panels':panels, 'note':'Same software-closed-loop run in every panel. Displayed FRS: raw position slices 0.5/1/1.5 s, not complete 3 s horizon; dashed circle is instantaneous combined radius, not predicted hard envelope. No physical/VINS validation.'}
    (a.output/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
    preview = Image.new('RGB', (900, 1650), 'white')
    for i in range(1, 4):
        im = Image.open(a.output/f'process_{i}.png'); im.thumbnail((900,550))
        preview.paste(im, (0, (i-1)*550))
    preview.save(a.output/'preview.png')

if __name__ == '__main__':
    main()
