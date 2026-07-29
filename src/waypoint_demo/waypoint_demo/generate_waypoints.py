#!/usr/bin/env python3
"""
生成 RTK Spline Planner 演示用 waypoint 文件。

路径包含：
  - Phase 1: 长直线 (X 方向, ~80m)
  - Phase 2: 左转弯弧线
  - Phase 3: 直线 (Y 方向, ~60m)
  - Phase 4: 右转弯弧线
  - Phase 5: 长直线 (X 方向, ~80m)
  - Phase 6: 折线 (zigzag)
  - Phase 7: 尾段直线

输出格式: x  y  theta (空格分隔, # 开头为注释)
"""

import math
import os
import sys


def compute_theta(pts):
    """根据相邻点计算每个点的航向角。"""
    thetas = []
    for i in range(len(pts)):
        if i < len(pts) - 1:
            dx = pts[i + 1][0] - pts[i][0]
            dy = pts[i + 1][1] - pts[i][1]
        else:
            dx = pts[i][0] - pts[i - 1][0]
            dy = pts[i][1] - pts[i - 1][1]
        thetas.append(math.atan2(dy, dx))
    return thetas


def generate_waypoints():
    """生成包含长直线和折线段的 waypoint 列表。"""
    pts = [
        # Phase 1: 长直线 (X 方向, 80m)
        (0.0,   0.0),
        (20.0,  0.0),
        (40.0,  0.0),
        (60.0,  0.0),
        (80.0,  0.0),

        # Phase 2: 左转弯
        (92.0,  3.0),
        (100.0, 10.0),
        (104.0, 20.0),

        # Phase 3: 直线 (Y 方向, 60m)
        (104.0, 40.0),
        (104.0, 60.0),
        (104.0, 80.0),

        # Phase 4: 右转弯
        (108.0, 92.0),
        (118.0, 98.0),
        (130.0, 100.0),

        # Phase 5: 长直线 (X 方向, 80m)
        (150.0, 100.0),
        (170.0, 100.0),
        (190.0, 100.0),
        (210.0, 100.0),

        # Phase 6: 折线 (zigzag)
        (220.0, 106.0),
        (230.0, 94.0),
        (240.0, 106.0),
        (250.0, 94.0),

        # Phase 7: 尾段直线
        (260.0, 100.0),
        (280.0, 100.0),
    ]

    thetas = compute_theta(pts)
    return [(p[0], p[1], t) for p, t in zip(pts, thetas)]


def write_waypoints(waypoints, filepath):
    """将 waypoints 写入 txt 文件。"""
    with open(filepath, 'w') as f:
        f.write('# RTK Spline Planner demo waypoints\n')
        f.write('# Format: x(m)  y(m)  theta(rad)\n')
        f.write('#\n')
        f.write('# Phase 1: Long straight (X, 80m)\n')
        for i, (x, y, theta) in enumerate(waypoints):
            if i == 5:
                f.write('# Phase 2: Left turn\n')
            elif i == 8:
                f.write('# Phase 3: Straight (Y, 60m)\n')
            elif i == 11:
                f.write('# Phase 4: Right turn\n')
            elif i == 14:
                f.write('# Phase 5: Long straight (X, 80m)\n')
            elif i == 18:
                f.write('# Phase 6: Zigzag\n')
            elif i == 22:
                f.write('# Phase 7: Final straight\n')
            f.write(f'{x:8.3f} {y:8.3f} {theta:8.4f}\n')

    print(f'Generated {len(waypoints)} waypoints -> {filepath}')


def main():
    if len(sys.argv) > 1:
        output = sys.argv[1]
    else:
        output = 'waypoints.txt'

    waypoints = generate_waypoints()
    write_waypoints(waypoints, output)


if __name__ == '__main__':
    main()
