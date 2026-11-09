"""지도/위치 — 지도 보기(터미널 지도 화면 MapPage) · 초기 위치(initialpose) 지정 · 목록 · 저장."""
from __future__ import annotations

from pathlib import Path

from ..defs import STAY
from ..menus import InfoItem, Menu, MenuItem
from .services import SaveMapItem


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
        if Path(self.path).is_file():   # 이 컴퓨터에 파일이 있으면 터미널 지도 화면으로 (웹 페이지 이동처럼 source를 넘김)
            self.ui.window.navigate('map', mode='view', source=self.path)
        print(f'{self.path}\n')
        print(self.bus.procs.read_map(self.path))
        print(f'\nNav로 쓰기: ./run-cli nav start {self.name}')
        return STAY


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
        return [MapShowItem(self.app, 'v', 'view'), MapShowItem(self.app, 'p', 'pose'), save] + (rows or [InfoItem(self.app, '저장된 지도 없음')])

    def center(self):
        return [f'[지도 폴더] {self.bus.procs.map_dir_txt}', ' <이름>/map.yaml + map.pgm', '',
                ' v) 지도 보기 · p) 초기 위치 지정 (amcl)', ' s) 지금 /map 저장 (SLAM 실행 중일 때)', ' 번호) 저장된 지도 보기 (+ yaml 내용)',
                '', f' 지도 /map: {self.state.map_txt()}']
