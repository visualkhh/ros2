"""calibration profile 로더 — 파일 경로에서 make_profile/cmd_at/total을 읽는다."""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def resolve(path: str | Path | None) -> Path | None:
    if not path:
        return None
    p = Path(path).expanduser()
    if not p.is_absolute():
        p = Path.cwd() / p
    if p.is_dir():
        p = p / "calib_profile.py"
    return p.resolve()


def load_profile(path: str | Path | None = None):
    """path=None이면 calibration/calib_default_profile.py."""
    if not path:
        import calib_default_profile  # same directory (02와 함께 실행)
        return calib_default_profile

    p = resolve(path)
    if p is None or not p.is_file():
        raise FileNotFoundError(f"profile 없음: {p}")
    name = f"calib_profile_user_{abs(hash(str(p)))}"
    spec = importlib.util.spec_from_file_location(name, p)
    if spec is None or spec.loader is None:
        raise ImportError(f"profile 로드 실패: {p}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    for attr in ("make_profile", "cmd_at", "total"):
        if not hasattr(mod, attr):
            raise AttributeError(f"{p}에 {attr}가 없음")
    mod.__profile_path__ = str(p)
    return mod
