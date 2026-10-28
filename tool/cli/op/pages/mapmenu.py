"""지도 파일 — 목록 · 저장."""
from __future__ import annotations

from ..defs import STAY
from .base import InfoItem, Menu, MenuItem
from .services import SaveMapItem


class MapInfoItem(MenuItem):
    def __init__(self, app, key: str, name: str, path):
        super().__init__(app)
        self.key = key
        self.name = name
        self.path = path

    def label(self):
        return self.name

    def run(self):
        print(f'{self.path}\n')
        print(self.bus.procs.read_map(self.path))
        print(f'\nNav로 쓰기: ./run-cli nav start {self.name}')
        return STAY


class MapMenu(Menu):
    title = '지도 파일'

    def items(self):
        save = SaveMapItem(self.app)
        save.key = 's'
        maps = self.bus.procs.maps()
        rows = [MapInfoItem(self.app, str(i), n, p) for i, (n, p) in enumerate(maps, 1)]
        return [save] + (rows or [InfoItem(self.app, '저장된 지도 없음')])

    def center(self):
        return [f'[지도 폴더] {self.bus.procs.map_dir_txt}', ' <이름>/map.yaml + map.pgm', '',
                ' s) 지금 /map 저장 (SLAM 실행 중일 때)', ' 번호) yaml 내용 보기',
                '', f' 지도 /map: {self.state.map_txt()}']
