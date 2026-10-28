"""수동 주행 — 텔레오퍼 + 한 번 이동 + 속도 설정."""
from __future__ import annotations

import argparse

from ..bus import CMD_VEL, MAX_ANG, MAX_LIN
from ..defs import STAY
from .base import Menu, MenuItem


class TeleopItem(MenuItem):
    """수동 주행 메뉴 · SLAM 맵핑 메뉴(t)가 같이 씀 — 키와 이름만 다름."""

    def __init__(self, app, key: str = '1', label: str = '텔레오퍼 (키보드)'):
        super().__init__(app)
        self.key = key
        self._label = label

    def label(self):
        return self._label

    def run(self):
        m = self.app.drive_cfg
        self.bus.cmd_teleop(argparse.Namespace(linear=m['linear'], angular=m['angular']))
        return STAY


class StepItem(MenuItem):
    def __init__(self, app, key: str, direction: str, label: str):
        super().__init__(app)
        self.key = key
        self.direction = direction
        self._label = label

    def label(self):
        m = self.app.drive_cfg
        return f'{self._label} ({m["duration"]:g}초)'

    def run(self):
        m = self.app.drive_cfg
        self.bus.cmd_drive(argparse.Namespace(direction=self.direction, speed=m['linear'],
                                              angular=m['angular'], duration=m['duration']))
        return STAY


class SpeedItem(MenuItem):
    def __init__(self, app, key: str, field: str, name: str, unit: str, maximum: float | None = None):
        super().__init__(app)
        self.maximum = maximum
        self.key = key
        self.field = field
        self.name = name
        self.unit = unit

    def label(self):
        return f'{self.name} = {self.app.drive_cfg[self.field]:g} {self.unit}'

    def run(self):
        value, ok = self.ui.read_float(f'{self.name} [{self.unit}]', self.app.drive_cfg[self.field],
                                       positive=True, maximum=self.maximum)
        if ok:
            self.app.drive_cfg[self.field] = value
        return STAY


class DriveMenu(Menu):
    title = '수동 주행'

    def items(self):
        return [TeleopItem(self.app),
                StepItem(self.app, '2', 'forward', '전진'), StepItem(self.app, '3', 'back', '후진'),
                StepItem(self.app, '4', 'left', '좌회전'), StepItem(self.app, '5', 'right', '우회전'),
                SpeedItem(self.app, '6', 'linear', '선속도', 'm/s', MAX_LIN),
                SpeedItem(self.app, '7', 'angular', '각속도', 'rad/s', MAX_ANG),
                SpeedItem(self.app, '8', 'duration', '한 번 이동 시간', 's')]

    def center(self):
        return [f'[수동 주행] → {CMD_VEL}',
                ' Nav가 주행 중이면 명령이 섞입니다 — 목표 취소 후 사용',
                ' 맵핑 땐 천천히: 0.10 m/s · 0.5 rad/s 정도', ''] + self.bus.status_lines()
