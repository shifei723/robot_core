#!/usr/bin/env python3
"""inspect_db_poses.py — 离线读 rtabmap 数据库里的里程计位姿，换算成 RPY。

用途: 判断传给 rtabmap 的 base frame 是否符合 REP-103(x前 y左 z上)。
      若相机水平前视时 roll/pitch 不在 0 附近而是 ±90°，说明 base frame
      是相机光学系(x右 y下 z前)，此时 Grid/MapFrameProjection 的水平化
      会把点云转歪，2D 栅格图必然错乱（3D 点云不受影响）。

用法: python3 inspect_db_poses.py <db_path>
"""
import sqlite3
import struct
import sys

import numpy as np

# 与 odom_to_tf.py 保持一致: body(光学系) → base_link(REP-103) 的纯旋转 (x,y,z,w)
Q_BODY_BASE = (0.5, -0.5, 0.5, 0.5)


def quat_to_mat(q):
    """四元数 (x,y,z,w) 转旋转矩阵。"""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def pose_from_blob(blob):
    """rtabmap 的位姿 blob 是 3x4 float32 行主序 [R|t]。"""
    v = struct.unpack('12f', blob[:48])
    return np.array(v, dtype=np.float64).reshape(3, 4)


def rpy_deg(R):
    """按 rtabmap/Eigen 的 ZYX(yaw-pitch-roll) 惯例分解。"""
    pitch = np.arcsin(-max(-1.0, min(1.0, R[2, 0])))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees([roll, pitch, yaw])


def main():
    db = sys.argv[1]
    con = sqlite3.connect(db)
    rows = con.execute('SELECT id, pose FROM Node WHERE pose IS NOT NULL '
                       'ORDER BY id').fetchall()
    con.close()
    if not rows:
        print('数据库里没有位姿')
        return

    print(f'节点数: {len(rows)}')
    print(f'{"id":>5} {"x":>8} {"y":>8} {"z":>8} '
          f'{"roll":>8} {"pitch":>8} {"yaw":>8}')
    picked = rows[:3] + rows[len(rows) // 2:len(rows) // 2 + 2] + rows[-3:]
    for nid, blob in picked:
        Rt = pose_from_blob(blob)
        r, p, y = rpy_deg(Rt[:, :3])
        t = Rt[:, 3]
        print(f'{nid:>5} {t[0]:>8.3f} {t[1]:>8.3f} {t[2]:>8.3f} '
              f'{r:>8.1f} {p:>8.1f} {y:>8.1f}')

    # 全体统计: 若 base frame 是光学系，roll/pitch 会整体钉在 ±90 附近
    allrpy = np.array([rpy_deg(pose_from_blob(b)[:, :3]) for _, b in rows])
    print('\n全部节点 roll/pitch/yaw 的中位数与范围:')
    for i, name in enumerate(('roll', 'pitch', 'yaw')):
        c = allrpy[:, i]
        print(f'  {name:>5}: 中位数 {np.median(c):>7.1f}°  '
              f'范围 [{c.min():.1f}, {c.max():.1f}]°')

    # 竖直方向判据: 位置的 z 分量若几乎不变，说明该轴确实是"高度"
    xyz = np.array([pose_from_blob(b)[:, 3] for _, b in rows])
    print('\n各轴位移范围(米): '
          + '  '.join(f'{a}={xyz[:, i].ptp():.2f}'
                      for i, a in enumerate('xyz')))

    # 验证修正: 右乘 body→base_link 的纯旋转，看 roll/pitch 能否回到 0 附近。
    # R_body_base 的三列 = base 的 x/y/z 轴在 body(光学系)中的表示，即 z、-x、-y。
    R_body_base = np.array([[0.0, -1.0, 0.0],
                            [0.0, 0.0, -1.0],
                            [1.0, 0.0, 0.0]])
    # 交叉校验 odom_to_tf.py 里实际用的四元数常量与上面的矩阵一致
    assert np.allclose(quat_to_mat(Q_BODY_BASE), R_body_base, atol=1e-9), \
        'Q_BODY_BASE 与 R_body_base 不一致，odom_to_tf.py 的常量可能写错了'

    fixed = np.array([rpy_deg(pose_from_blob(b)[:, :3] @ R_body_base)
                      for _, b in rows])
    print('\n右乘 body→base_link 后(即 rtabmap 实际会用到的 REP-103 位姿):')
    for i, name in enumerate(('roll', 'pitch', 'yaw')):
        c = fixed[:, i]
        print(f'  {name:>5}: 中位数 {np.median(c):>7.1f}°  '
              f'范围 [{c.min():.1f}, {c.max():.1f}]°')
    print('  判据: roll/pitch 应在 0 附近(手持倾斜量级)，yaw 为朝向不限')


if __name__ == '__main__':
    main()
