"""지도/위치 — 지도 보기(터미널 지도 화면 MapPage) · 초기 위치(initialpose) 지정 · 목록 · 저장."""
from __future__ import annotations

from pathlib import Path

from ..defs import STAY
from ..menus import InfoItem, Menu, MenuItem
from .services import ChangeMapItem, NavMapItem, SaveMapItem


class MapInfoItem(MenuItem):
    def __init__(self, app, key: str, name: str, path):
        super().__init__(app)
        self.key = key
        self.name = name
        self.path = path

    def label(self):
        return self.name

    shows_busy = False   # 지도 화면을 연다

    def run(self):
        # 원본은 로봇의 ros2_ws/map/<이름>. PC면 MapPage가 필요할 때만 캐시로 내려받는다.
        return self.ui.window.navigate('map', mode='view', source=self.path)


class MapShowItem(MenuItem):
    """지도 화면(MapPage) — 'view': 지금 /map 보기 · 'pose': 지도 위에서 초기 위치(initialpose) 지정."""
    shows_busy = False

    def __init__(self, app, key: str, mode: str):
        super().__init__(app)
        self.key, self._mode = key, mode

    def label(self):
        return '지도 보기 (지금 /map · 로봇 위치)' if self._mode == 'view' else '초기 위치 지정 (지도에서 찍기 → /initialpose)'

    def run(self):
        if self._mode == 'view':
            return self.ui.window.navigate('map', mode='view')
        return self.ui.window.navigate('map', mode='pose', on_pose=self._pose)

    def _pose(self, x: float, y: float, yaw_deg: float) -> None:
        if self.bus.set_initial_pose(x, y, yaw_deg) != 0:
            raise RuntimeError('amcl 응답 없음 — Nav(지도 주행)가 떠 있는지 확인')


class MapMenu(Menu):
    title = '지도/위치'

    def items(self):
        save = SaveMapItem(self.app)
        save.key = 's'
        maps = self.bus.procs.maps()
        rows = [MapInfoItem(self.app, str(i), n, p) for i, (n, p) in enumerate(maps, 1)]
        return [MapShowItem(self.app, 'v', 'view'), MapShowItem(self.app, 'p', 'pose'),
                NavMapItem(self.app, 'g'),
                ChangeMapItem(self.app, 'n', always=True), save] + (
                    rows or [InfoItem(self.app, '저장된 지도 없음')])

    def center(self):
        return [f'[지도 폴더] {self.bus.procs.map_dir_txt}', ' <이름>/map.yaml + map.pgm', '',
                ' v) 지금 /map 보기 · g) 목표·초기위치·취소 (한 화면)',
                ' n) Nav 지도 바꾸기 (다른 지도 선택 → Nav 재시작)',
                ' p) 초기 위치 지정 (amcl) · s) 지금 /map 저장',
                ' 번호) 저장된 지도 보기 (PC는 로봇에서 내려받아 표시)',
                '', f' 지도 /map: {self.state.map_txt()}']
