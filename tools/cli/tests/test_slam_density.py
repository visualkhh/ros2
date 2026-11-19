#!/usr/bin/env python3
"""SLAM 조밀도 설정 시험 — ROS 없이: python3 tools/cli/tests/test_slam_density.py

확인: ① 맵 해상도(launch resolution)와 Cartographer 서브맵 격자 해상도가 같은 값 (다르면 /map이 서브맵을 다시 그려 뭉개짐)
② 이동·회전 필터와 점 필터가 Cartographer 기본값(0.2 m · 1° · 0.025 m)보다 촘촘 ③ 맵 해상도가 기본 0.05보다 작음
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3] / 'ros2_ws/src/tb3_bringup'
lua = (ROOT / 'config/cartographer_2d.lua').read_text()
launch = (ROOT / 'launch/cartographer.launch.py').read_text()


def lua_num(key):
    m = re.search(rf'^{re.escape(key)}\s*=\s*([^\s-][^-\n]*?)\s*(--.*)?$', lua, re.M)
    assert m, key
    expr = m.group(1).strip()
    m2 = re.fullmatch(r'math\.rad\(([\d.]+)\)', expr)
    return float(m2.group(1)) if m2 else float(expr)   # 각도는 도(°) 그대로


res_launch = float(re.search(r"DeclareLaunchArgument\('resolution', default_value='([\d.]+)'\)", launch).group(1))
res_grid = lua_num('TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution')
assert res_launch == res_grid, (res_launch, res_grid)                                  # ①
assert res_launch < 0.05                                                                # ③
assert lua_num('TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters') < 0.2         # ②
assert lua_num('TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians') < 1.0
assert lua_num('TRAJECTORY_BUILDER_2D.voxel_filter_size') < 0.025
print('PASS test_slam_density')
