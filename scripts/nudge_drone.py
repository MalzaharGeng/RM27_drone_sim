#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""让仿真里的无人机做一段平滑小位移，用于验证 SLAM ↔ 真值的坐标映射。

为什么不用 GUI 拖动：
    GUI 的 transform gizmo 每次拖动都是一次 set_pose 瞬移。幅度稍大时
    ORB_SLAM3 会跟丢并重新初始化，新地图的世界系以「跳变后第一帧」为原点，
    于是 SLAM 位置归零、真值已经偏出几米，比对结果会显示成"错了几米"——
    那是假象，不是坐标转换错误。

这里改成小步连续位移（默认 2 cm/步 × 10 Hz ≈ 0.2 m/s），ORB 能正常跟踪，
同时每步都重设 z，避免模型因重力下坠。

用法：
    python3 nudge_drone.py                          # 沿 +x 平移 2 m，约 10 s
    python3 nudge_drone.py --axis y --dist -2       # 沿 -y 平移 2 m
    python3 nudge_drone.py --dist 1 --duration 5 --hz 20
    python3 nudge_drone.py --model x500_mono_cam_0 --world rm27_world --start 5 6 0.5
"""
import argparse
import subprocess
import sys
import time


def set_pose(world, model, x, y, z):
    req = (f'name: "{model}", '
           f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
           f'orientation: {{w: 1.0}}')
    cmd = [
        'gz', 'service',
        '-s', f'/world/{world}/set_pose',
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '1000',
        '--req', req,
    ]
    r = subprocess.run(cmd, capture_output=True, text=True)
    return r.returncode == 0 and 'true' in r.stdout.lower()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='rm27_world')
    ap.add_argument('--model', default='x500_mono_cam_0')
    ap.add_argument('--start', nargs=3, type=float, default=[5.0, 6.0, 0.5],
                    metavar=('X', 'Y', 'Z'), help='起始位姿（与 PX4_GZ_MODEL_POSE 一致）')
    ap.add_argument('--axis', choices=['x', 'y', 'z'], default='x')
    ap.add_argument('--dist', type=float, default=2.0, help='位移距离（米，可负）')
    ap.add_argument('--duration', type=float, default=10.0, help='总时长（秒）')
    ap.add_argument('--hz', type=float, default=10.0, help='更新频率（Hz）')
    args = ap.parse_args()

    n = max(int(args.duration * args.hz), 2)
    step = args.dist / (n - 1)
    period = args.duration / (n - 1)

    x0, y0, z0 = args.start
    print(f'起点 ({x0:.2f}, {y0:.2f}, {z0:.2f})  轴向 {args.axis}  '
          f'距离 {args.dist:.2f} m  {n} 步 @ {args.hz:.0f} Hz  '
          f'(约 {abs(args.dist / args.duration):.2f} m/s)')

    fails = 0
    t_start = time.time()
    for i in range(n):
        d = step * i
        x, y, z = x0, y0, z0
        if args.axis == 'x':
            x += d
        elif args.axis == 'y':
            y += d
        else:
            z += d
        if not set_pose(args.world, args.model, x, y, z):
            fails += 1
        if (i + 1) % max(int(args.hz), 1) == 0:
            print(f'  {i + 1:4d}/{n}  pose=({x:+.2f}, {y:+.2f}, {z:+.2f})')
        # 用绝对时间对齐，避免累积漂移
        nxt = t_start + period * (i + 1)
        time.sleep(max(0.0, nxt - time.time()))

    total = time.time() - t_start
    print(f'完成：{total:.1f} s，失败 {fails} 次')
    if fails:
        print('有 set_pose 失败：确认 world/model 名字正确（gz service -l | grep set_pose）',
              file=sys.stderr)
        return 1
    print('现在看 slam_pose_to_px4.py --compare：'
          f'SLAM NED 应朝对应方向变化约 {args.dist:+.1f} m')
    return 0


if __name__ == '__main__':
    sys.exit(main())
