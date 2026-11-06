#!/usr/bin/env python3
"""cli-test — ROS 없이 visualkhh robot_tool UI/UX 시험.

  ./tool/run-cli-test status
  ./tool/run-cli-test nodes
  ./tool/run-cli-test node-status /controller_server
  ./tool/run-cli-test ab nodes
  ./tool/run-cli-test ab rl_nav_controller status
  ./tool/run-cli-test ab rl_nav_controller on
  ./tool/run-cli-test menu        # 대화형 메뉴 (터미널)
  # guide는 rosbag/Nav2가 필요해 cli-test에서는 실행하지 않는다.

가짜 ROS층(fake_ros)으로 진짜 robot_tool 코드를 그대로 돌린다.
발행은 [PUB ...]으로 찍히고, /cmd_manual_*는 가짜 state_manager가 받아 상태를 바꾼다.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'cli'))
sys.path.insert(0, str(HERE))

import fake_ros  # noqa: E402

fake_ros.install()

import robot_tool as rt  # noqa: E402


class FakeProcs:
    """launch/SSH 없이 프로세스 상태만 흉내낸다."""

    host = 'fake'
    map_dir_txt = 'fake/map'

    def running(self, name):
        return name == 'base'

    def pid(self, name):
        return 4242 if name == 'base' else None

    def uptime(self, name):
        return 60.0

    def info(self, name):
        return None

    def maps(self):
        return []

    def resolve_map(self, name):
        return None

    def save_map(self, name):
        return True, '저장(fake)'

    def start(self, name, map_yaml=None):
        return True, f'{name}: 시작(fake)'

    def stop(self, name):
        return True, f'{name}: 종료(fake)'

    def restart(self, name):
        return True, f'{name}: 재시작(fake)'

    def restart_all(self):
        return True, '전체 재시작(fake)'

    def tail(self, name, n=40):
        return '(fake 로그 없음)'

    def build(self, extra=None, full=False):
        print(f'[BUILD] {extra or []} full={full}')
        return 0

    def node_kill(self, name, force=False):
        return True, f'{name}: 중지(fake)'

    def node_restart(self, name, force=False):
        return True, f'{name}: 재시작(fake)'

    def node_cleanup(self):
        return '정리(fake)'


def _seed(app: rt.RobotTool):
    st = app.state
    st.odom.tick()
    st.scan.tick()
    st.map.tick()
    st.amcl_rate.tick()
    st.cmd_vel_nav.tick()
    st.pose = (1.0, 2.0, 0.5)
    st.vel = (0.10, 0.20)
    st.scan_info = '360점 · 0.12~3.5m'
    st.map_info = (100, 100, 0.05)
    st.map_known = 0.60
    st.amcl = (1.0, 2.0, 0.5)
    st.on_state('control', 'NAV_PLAN')
    st.on_state('rl', False)
    st.on_state('estop', False)
    st.on_state('operation', 'LOCALIZATION')
    st.on_state('map', 'office')
    st.on_diagnostics(fake_ros.DiagnosticArray(status=[
        fake_ros.DiagnosticStatus(
            level=0, name='opencr', message='정상',
            values=[fake_ros.KeyValue('battery_V', '11.8'),
                    fake_ros.KeyValue('battery_pct', '60'),
                    fake_ros.KeyValue('left_temp_C', '38'),
                    fake_ros.KeyValue('right_temp_C', '39'),
                    fake_ros.KeyValue('left_current_A', '0.12'),
                    fake_ros.KeyValue('right_current_A', '0.13')]),
    ]))
    st.on_joint_states(fake_ros.JointState(
        name=['wheel_left_joint', 'wheel_right_joint'],
        position=[1.2, 1.3], velocity=[0.5, 0.5], effort=[0.12, 0.13]))
    st.rl_status = [0, 2, 1, 0, 1, 0.05, 2.0, 0.0, 0.0, 1]  # DWB, enable=false
    st.rl_status_time = time.monotonic()
    st.goal_status = '목표 없음 (fake)'


_orig_init = rt.RobotTool.__init__


def _init(self):
    _orig_init(self)
    _seed(self)


rt.make_procs = lambda node: FakeProcs()
rt.RobotTool.poll = lambda self, seconds=0.2: None
rt.RobotTool.__init__ = _init


if __name__ == '__main__':
    raise SystemExit(rt.main())
