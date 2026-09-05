#!/usr/bin/env python3
"""统计平墙上各 ring 的单帧厚度和跨帧距离波动。"""
import argparse
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

def view(msg, name, dtype):
    f = next((v for v in msg.fields if v.name == name), None)
    if f is None:
        raise ValueError(f"missing field {name}")
    return np.ndarray(msg.width * msg.height, dtype=dtype, buffer=msg.data,
                      offset=f.offset, strides=(msg.point_step,))

class Analyzer(Node):
    def __init__(self, args):
        super().__init__('ring_wall_stats')
        self.a, self.frame, self.ref, self.hist = args, 0, None, {}
        self.rng = np.random.default_rng(7)
        self.started = time.monotonic()
        self.create_subscription(PointCloud2, args.topic, self.callback,
                                 qos_profile_sensor_data)

    def roi(self, xyz):
        a = self.a
        if a.forward_axis == 'y':
            forward, lateral = xyz[:,1], xyz[:,0]
        else:
            forward, lateral = xyz[:,0], xyz[:,1]
        return ((forward >= a.forward_min) & (forward <= a.forward_max) &
                (np.abs(lateral) <= a.lateral_abs) &
                (xyz[:,2] >= a.z_min) & (xyz[:,2] <= a.z_max))

    def wall(self, xyz):
        a = self.a
        mask = self.roi(xyz)
        pts, idx = xyz[mask], np.flatnonzero(mask)
        if len(pts) < a.min_points:
            return None
        best = None
        for _ in range(a.ransac_iters):
            s = pts[self.rng.choice(len(pts), 3, replace=False)]
            n = np.cross(s[1]-s[0], s[2]-s[0]); norm = np.linalg.norm(n)
            if norm < 1e-6:
                continue
            n /= norm
            if abs(n[2]) > a.max_normal_z:
                continue
            d = -float(n @ s[0]); inside = np.abs(pts @ n + d) <= a.threshold
            score = int(inside.sum())
            if best is None or score > best[0]:
                best = score, inside
        if best is None or best[0] < a.min_points:
            return None
        selected = pts[best[1]]; center = selected.mean(axis=0)
        _, _, vh = np.linalg.svd(selected-center, full_matrices=False); n = vh[-1]
        if n[0] < 0: n = -n
        d = -float(n @ center)
        inside = np.abs(pts @ n + d) <= a.threshold
        return n, d, idx[inside]

    def callback(self, msg):
        try:
            xyz = np.column_stack((view(msg,'x','<f4'), view(msg,'y','<f4'),
                                   view(msg,'z','<f4'))).astype(np.float64)
            ring = view(msg,'ring','<u2')
        except ValueError as e:
            self.get_logger().error(str(e)); return
        finite = np.isfinite(xyz).all(axis=1); xyz, ring = xyz[finite], ring[finite]
        if self.ref is None:
            result = self.wall(xyz)
            if result is None:
                self.get_logger().warning('未找到足够大的前方垂直平面', throttle_duration_sec=2.0); return
            n, d, idx = result
            self.ref = n, d
            self.get_logger().info(f'锁定墙面 points={len(idx)} distance={abs(d):.3f}m normal={n}')
        rn, rd = self.ref
        a = self.a
        roi = self.roi(xyz)
        signed = xyz @ rn + rd
        # 后续帧固定使用首帧参考面，禁止 RANSAC 跳到场景中的另一堵墙。
        idx = np.flatnonzero(roi & (np.abs(signed) <= a.track_threshold))
        if len(idx) < a.min_points:
            self.get_logger().warning('固定参考墙面附近点数不足', throttle_duration_sec=2.0); return
        residual = signed[idx] * 1000.; rings = ring[idx]
        rows = []
        for rid in np.unique(rings):
            v = residual[rings == rid]
            if len(v) < self.a.min_ring_points: continue
            p10, med, p90 = np.percentile(v, [10,50,90]); std = v.std()
            rows.append((int(rid),len(v),med,std,p10,p90)); self.hist.setdefault(int(rid),[]).append(med)
        if not rows: return
        self.frame += 1
        shift = np.median([v[2] for v in rows]); width = np.median([v[5]-v[4] for v in rows])
        print(f'\nframe={self.frame} stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} '
              f'points={len(idx)} rings={len(rows)} shift={shift:+.1f}mm median_width={width:.1f}mm')
        print('ring count median_mm std_mm p10_mm p90_mm width_mm')
        for rid,c,med,std,p10,p90 in rows:
            print(f'{rid:4d} {c:5d} {med:+9.1f} {std:6.1f} {p10:+7.1f} {p90:+7.1f} {p90-p10:8.1f}')
        if self.frame % self.a.summary_every == 0:
            print('\nCROSS_FRAME ring frames mean_mm std_mm min_mm max_mm peak_to_peak_mm')
            for rid in sorted(self.hist):
                v=np.asarray(self.hist[rid][-self.a.summary_every:])
                if len(v) >= max(3,self.a.summary_every//2):
                    print(f'{rid:4d} {len(v):6d} {v.mean():+7.1f} {v.std():6.1f} {v.min():+7.1f} {v.max():+7.1f} {np.ptp(v):14.1f}')
        if self.a.frames and self.frame >= self.a.frames:
            self.get_logger().info(f'完成 {self.frame} 帧，用时 {time.monotonic()-self.started:.1f}s')
            rclpy.shutdown()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--topic',default='/nuc/rslidar_points_front'); p.add_argument('--frames',type=int,default=20)
    p.add_argument('--forward-axis',choices=('x','y'),default='y',help='Airy 原始坐标默认 +Y 为前方')
    p.add_argument('--forward-min',type=float,default=.3); p.add_argument('--forward-max',type=float,default=5.)
    p.add_argument('--lateral-abs',type=float,default=1.5); p.add_argument('--z-min',type=float,default=-.5); p.add_argument('--z-max',type=float,default=1.5)
    p.add_argument('--threshold',type=float,default=.10); p.add_argument('--ransac-iters',type=int,default=100)
    p.add_argument('--track-threshold',type=float,default=.12,help='锁定后允许统计的墙面法向范围')
    p.add_argument('--max-normal-z',type=float,default=.35); p.add_argument('--min-points',type=int,default=300)
    p.add_argument('--min-ring-points',type=int,default=8); p.add_argument('--summary-every',type=int,default=10)
    a=p.parse_args(); rclpy.init(); node=Analyzer(a)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
