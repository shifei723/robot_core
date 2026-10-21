#!/usr/bin/env python3
"""Evaluate sensor-driven runs, retaining failed trials instead of selecting successes."""
import argparse,json
from pathlib import Path
import numpy as np


def evaluate(root):
    s=json.loads((root/'summary.json').read_text());r=json.loads((root/'trace.json').read_text())
    frames=np.load(root/'snapshots.npz',allow_pickle=True)['frames']
    s['run']=root.name
    stats=r.get('perception_stats',[])
    if stats:
        s['background_point_fraction']=sum(a['known_static_points'] for a in stats)/sum(a['input_points'] for a in stats)
        s['background_and_tracking_ms_p95']=float(np.percentile([a['filter_and_tracking_ms'] for a in stats],95))
    s['masked_snapshots']=int(sum(f['filtered'] is not None and np.any(f['filtered']==65535) for f in frames))
    s['max_map_interval_s']=max([b['t']-a['t'] for a,b in zip(r['maps'][:-1],r['maps'][1:])],default=0.)
    s['final_control_state']=r['poses'][-1]['status'] if r['poses'] else ''
    s['max_map_points']=max([a['points'] for a in r['maps']],default=0)
    ages=[a['age'] for a in r['tracks'] if a['healthy']]
    s['healthy_track_age_p95_ms']=float(np.percentile(ages,95)*1000) if ages else None
    s['states']=dict(zip(*np.unique([a['status'] for a in r['poses']],return_counts=True)))
    s['states']={k:int(v) for k,v in s['states'].items()}
    detected=[a['t'] for a in r['tracks'] if any(o['state']==1 and o['observed'] for o in a['objects'])]
    s['first_dynamic_detection_s']=min(detected) if detected else None
    s['pipeline_evidence']=bool(s.get('track_messages',0)>10 and s.get('map_messages',0)>10 and s.get('plans',0)>0 and s.get('decision_messages',0)>0)
    if s['case']=='dropout':
        onset=min(a['t'] for a in r['sensors'] if not a['published'])
        pre=[a['v'] for a in r['poses'] if onset-.4<a['t']<onset]
        post=[a for a in r['poses'] if a['t']>onset]
        s['dropout_at_s']=onset;s['pre_dropout_max_speed']=max(pre,default=0)
        last_moving=max([a['t'] for a in post if abs(a['v'])>.001],default=onset)
        s['stop_latency_s']=max(0,last_moving-onset)
        s['pass']=bool(s['pipeline_evidence'] and not s['collision'] and s['error'] is None and s['pre_dropout_max_speed']>.01 and s['stop_latency_s']<.7)
        s['criterion']='moving before dropout; stop within 0.7 s and remain stopped; no collision'
    else:
        s['pass']=bool(s['pipeline_evidence'] and s['reached'] and not s['collision'] and s['error'] is None)
        if s['case']!='static':s['pass']=s['pass'] and bool(detected) and s['masked_snapshots']>0 and (s['frs_waits']+s['frs_slow'])>0
        s['criterion']='all stages active; goal error <0.16 m (plus stopped/GOAL_REACHED when declared by run); no collision; dynamic scenarios additionally require detected motion, mask and FRS intervention'
    if stats and 'online_background_points' in stats[0]:
        s['online_max_points']=max(a['online_background_points'] for a in stats)
        s['online_promotions']=max(a['online_promotions'] for a in stats)
        s['online_cleared_points']=max(a['online_cleared_total'] for a in stats)
        s['online_reactivations']=max(a.get('online_reactivations',0) for a in stats)
    if s['case']=='stop_restart':
        s['promoted_before_motion']=any(a['t']<6 and a.get('online_promotions',0)>0 for a in stats)
        s['false_dynamic_before_motion']=any(t<6 for t in detected)
        s['pass']=bool(s['pass'] and s['promoted_before_motion'] and not s['false_dynamic_before_motion'] and s.get('online_cleared_points',0)>0)
        s['criterion']+='; promoted before t=6, no dynamic label before true motion, old online points cleared'
    (root/'evaluation.json').write_text(json.dumps(s,indent=2,ensure_ascii=False))
    return s


def main():
    p=argparse.ArgumentParser();p.add_argument('base',type=Path);p.add_argument('--runs',nargs='+',required=True);p.add_argument('--output-prefix',default='validation_report');args=p.parse_args()
    rows=[evaluate(args.base/run) for run in args.runs]
    with_prior=any(s.get('prebuilt_static_map',False) for s in rows)
    report={'scope':'Sensor-driven software integration with prebuilt STATIC PCD when enabled. Ideal pose, noiseless raycasting, planar kinematic plant. Not a D435i/VINS or physics/contact validation.',
            'all_pass':all(s['pass'] for s in rows),'runs':rows,
            'limitations':['Single deterministic run per case; no statistical success-rate claim.',
              'Measured points determine clusters and motion; no ground-truth obstacle states are published to perception or FRS.',
              'Failure probes remain in sibling directories; reported cases are listed explicitly.',
              'Long occlusion, changing visible wall patches and static/dynamic transitions remain robustness risks.']}
    (args.base/(args.output_prefix+'.json')).write_text(json.dumps(report,indent=2,ensure_ascii=False))
    lines=['深度感知—在线建图—FRS—导航：无头闭环验证','',
           ('输入：预建静态 PCD、模拟深度、CameraInfo、理想位姿/TF；只对背景外点聚类跟踪。' if with_prior else '输入：模拟深度、CameraInfo、理想位姿/TF。感知节点自行聚类、跟踪、估速。'),
           '被测：实际 ROS 感知节点、GridMap、SCAN-Planner、闭环控制器。',
           '模型：无噪声针孔深度射线、平面运动学及腿高响应；不包含 VINS 估计或接触动力学。','',
           '验收结果：'+('全部通过' if report['all_pass'] else '存在未通过场景，不能宣布动态导航整体验收通过'),'']
    for s in rows:
        lines.extend([s['run']+'：'+('通过' if s['pass'] else '未通过'),
          f"  到达目标={s['reached']}；包络碰撞={s['collision']}；最小净距={s['min_clearance']:.3f} m",
          f"  规划次数={s['plans']}；地图消息={s['map_messages']}；FRS 决策消息={s['decision_messages']}",
          f"  最大地图发布间隔={s['max_map_interval_s']:.3f} s；结束控制状态={s['final_control_state']}",
          f"  动态掩码快照={s['masked_snapshots']}；健康目标消息延时 P95={s['healthy_track_age_p95_ms']:.1f} ms"])
        if s['case']=='dropout':lines.append(f"  中断前速度={s['pre_dropout_max_speed']:.3f} m/s；停车延迟约={s['stop_latency_s']:.3f} s")
        if s.get('arrival_time') is not None:lines.append(f"  从仿真启动计到达时间={s['arrival_time']:.2f} s")
        lines.append('')
    lines.extend(['单次场景验证不等于统计成功率。失败日志与原始轨迹均保留。',
      ('部署待验证：定位误差与底图变化、目标分裂/遮挡、多目标复杂交互；未进行实机 VINS 测试。' if with_prior else '当前关键遗留：转向时墙面可见区域变化造成误跟踪；目标分裂/丢失后的遮挡处理及自主恢复。')])
    (args.base/(args.output_prefix+'.txt')).write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))

if __name__=='__main__':main()
