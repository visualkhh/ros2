"""BaseReport — 측정 폴더(<폴더>/raw.csv)를 받아 보고서를 만드는 바탕. 표준 라이브러리만.

  python3 <보고서.py> <폴더>          → single(폴더)          그 폴더 하나 분석
  python3 <보고서.py> <폴더A> <폴더B>  → compare(폴더A, 폴더B)  두 폴더 비교 (A = 기준: sim · before …)
무엇을 계산하고 그릴지는 하위 클래스가 정한다 (raw.csv 열도 task마다 다르다).
"""
from __future__ import annotations

import argparse
import csv
from abc import ABC, abstractmethod
from pathlib import Path


class BaseReport(ABC):
    @staticmethod
    def run_dir(path: str) -> Path:
        """측정 폴더 또는 그 안의 raw.csv 경로 → 폴더 (raw.csv가 없으면 SystemExit)."""
        p = Path(path)
        d = p.parent if p.is_file() else p
        if not (d / "raw.csv").is_file():
            raise SystemExit(f"raw.csv 없음: {d}")
        return d

    @staticmethod
    def load(path) -> list[dict]:
        """raw.csv(또는 폴더) → 줄 dict 목록. 숫자는 float, 문자열(rl_onnx 등)은 그대로, t는 0부터."""
        p = Path(path)
        p = p / "raw.csv" if p.is_dir() else p
        def value(v):
            try:
                return float(v)
            except (TypeError, ValueError):
                return v
        rows = [{k: value(v) for k, v in r.items()} for r in csv.DictReader(open(p))]
        if not rows:
            raise SystemExit(f"빈 CSV: {p}")
        t0 = rows[0]["t"]
        for r in rows:
            r["t"] -= t0
        return rows

    @abstractmethod
    def single(self, run_dir: Path):
        """폴더 하나 분석 — 결과 파일은 그 폴더에."""

    @abstractmethod
    def compare(self, a: Path, b: Path, out: str | None = None, show: bool = False):
        """두 폴더 비교 — 기본 결과 위치는 b 폴더."""

    def main(self, argv=None) -> None:
        p = argparse.ArgumentParser(description=(type(self).__doc__ or "").strip())
        p.add_argument("run", nargs="+", help="측정 폴더(또는 raw.csv) 1개 = 단건 · 2개 = 비교 (앞이 기준)")
        p.add_argument("--out", default=None, help="비교 그림 경로 (기본: 뒤 폴더 안)")
        p.add_argument("--show", action="store_true", help="저장 후 창으로 띄우기")
        args = p.parse_args(argv)
        if len(args.run) == 1:
            self.single(self.run_dir(args.run[0]))
        elif len(args.run) == 2:
            self.compare(self.run_dir(args.run[0]), self.run_dir(args.run[1]), args.out, args.show)
        else:
            p.error("측정 폴더 1개(단건) 또는 2개(비교)")
