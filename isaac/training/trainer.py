"""학습 실행기 + 공통 베이스.

`python -m training <task> [옵션...]` → `Train.main()`이 task 폴더(`train.py`가 있는 폴더)를 찾아
그 모듈의 `main(argv)`로 넘긴다. task의 `train.py`는 `Train`을 상속해서
`add_args` / `before_launch` / `run`만 구현하고, `def main(argv=None): return TaskTrain.main(argv)`를 둔다.
`main`은 하나다: 베이스(`Train.main`)로 부르면 task 찾기, 서브클래스로 부르면 그 task 학습.

공통으로 해 주는 것:
- `--watch` → `--visualizer kit` (기본 headless)
- AppLauncher 띄우고 닫기
- 결과 폴더 `artifacts/training/<시각>-<exp-name>/` (`self.run_dir`)
- `--watch`일 때 `SCREENSHOT_KEY`(기본 P) → `run_dir/screen_shot_<시각>.png` (뷰포트 캡처)
- `--snap SEC` → headless에서도 SEC초마다 `run_dir/snaps/<시각>.png` + `run_dir/latest.png`(덮어씀)
  — task `run()`이 env cfg에 `add_snap_visualizer`, env 만든 뒤 `install_snap`을 부른다.
"""
from __future__ import annotations

import argparse
import importlib
import sys
from abc import ABC, abstractmethod
from collections.abc import Sequence
from datetime import datetime
from pathlib import Path

TRAINING = Path(__file__).resolve().parent
ISAAC = TRAINING.parent
RUNS_DIR = ISAAC / "artifacts" / "training"   # calibration은 artifacts/calibration


class Train(ABC):
    """task 학습 실행 단위. 인스턴스는 AppLauncher가 뜬 뒤에 만들어진다."""

    NAME = "train"          # 결과 폴더 / ONNX 기본 이름 (`--exp-name`)
    SCREENSHOT_KEY = "P"    # carb.input.KeyboardInput 이름

    def __init__(self, args):
        self.args = args
        self.watch = "kit" in (getattr(args, "visualizer", None) or [])
        self.run_dir = RUNS_DIR / f"{datetime.now():%Y%m%d_%H%M%S}-{args.exp_name}"
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self._key_sub = None   # 구독 핸들 — 들고 있어야 유지된다
        self.env = None        # run()이 채운다 — 스크린샷이 visualizer에 접근할 때
        self._snap_at = 0.0

    @classmethod
    def add_args(cls, parser: argparse.ArgumentParser) -> None:
        """task 옵션 추가."""

    @classmethod
    def before_launch(cls, args, parser: argparse.ArgumentParser) -> None:
        """sim 띄우기 전 검사·로딩 (잘못된 입력이면 parser.error로 바로 실패)."""

    @abstractmethod
    def run(self) -> None:
        """env 만들고 학습. 결과는 `self.run_dir`에."""
        raise NotImplementedError

    def add_snap_visualizer(self, env_cfg) -> None:
        """--snap인데 창(--watch)이 없으면 화면 없는 Kit visualizer를 붙인다 (격자 가운데를 비스듬히)."""
        if not self.args.snap or self.watch:
            return
        from isaaclab_visualizers.kit import KitVisualizerCfg

        env_cfg.sim.visualizer_cfgs = list(getattr(env_cfg.sim, "visualizer_cfgs", None) or []) + [
            # env 격자는 월드 원점 중심 (env 0은 모서리) → 원점을 비스듬히 내려다봄 = 가운데 3×3 env 정도
            KitVisualizerCfg(headless=True, origin_type="world", eye=(6.0, -6.0, 5.0), lookat=(0.0, 0.0, 0.0))]

    def install_snap(self, raw_env) -> None:
        """env.step 뒤에 시간을 보고 --snap 초마다 저장. 학습 루프는 runner.learn()이 쥐고 있어 step에 건다."""
        if not self.args.snap:
            return
        import shutil
        import time

        snaps = self.run_dir / "snaps"
        snaps.mkdir(exist_ok=True)
        step = raw_env.step

        def step_and_snap(action):
            out = step(action)
            now = time.monotonic()
            if now - self._snap_at >= self.args.snap:
                self._snap_at = now
                path = self.screenshot(snaps / f"{datetime.now():%H%M%S}.png")
                if path and path.is_file():
                    shutil.copyfile(path, self.run_dir / "latest.png")
            return out

        raw_env.step = step_and_snap
        print(f"[train] --snap {self.args.snap:g}s → {snaps}/ · 최신: {self.run_dir / 'latest.png'}", flush=True)

    def screenshot(self, path: Path | None = None) -> Path | None:
        """현재 화면 → run_dir/screen_shot_<시각>.png.

        IsaacLab 6.x `kit` visualizer는 `render_rgb_array()`로 RGB를 바로 준다.
        예전 `omni.kit.viewport.utility` 캡처는 비동기라 학습 루프에서 파일이 안 생길 수 있어
        visualizer 캡처를 먼저 시도하고, 안 되면 viewport 캡처로 넘어간다.
        """
        path = path or self.run_dir / f"screen_shot_{datetime.now():%Y%m%d_%H%M%S}.png"
        env = getattr(self, "env", None)
        sim = getattr(getattr(env, "unwrapped", None), "sim", None)
        for viz in getattr(sim, "_visualizers", None) or []:
            fn = getattr(viz, "render_rgb_array", None)
            if fn is None:
                continue
            try:
                rgb = fn()
                if rgb is not None and getattr(rgb, "size", 0) and int(rgb.max()) > 0:
                    from PIL import Image
                    Image.fromarray(rgb).save(path)
                    print(f"[train] screenshot → {path}", flush=True)
                    return path
            except Exception as exc:
                print(f"[train] screenshot visualizer 실패: {exc}", flush=True)

        if not self.watch:   # 창 없음 — viewport 캡처 대상이 없다
            return None
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

        capture_viewport_to_file(get_active_viewport(), str(path))
        print(f"[train] screenshot (viewport) → {path}", flush=True)
        return path

    def bind_screenshot_key(self) -> None:
        # 학습 루프는 runner.learn()이 쥐고 있어 키를 폴링할 곳이 없다 → 이벤트 구독
        import carb.input
        import omni.appwindow

        getattr(carb.input.KeyboardInput, self.SCREENSHOT_KEY)   # 이름 검증

        def on_key(event, *_):
            if event.type == carb.input.KeyboardEventType.KEY_PRESS and event.input.name == self.SCREENSHOT_KEY:
                self.screenshot()
            return True

        keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self._key_sub = carb.input.acquire_input_interface().subscribe_to_keyboard_events(keyboard, on_key)
        print(f"[train] --watch: {self.SCREENSHOT_KEY} 키 → 스크린샷 ({self.run_dir})", flush=True)

    @staticmethod
    def tasks() -> list[str]:
        return sorted(p.parent.name for p in TRAINING.glob("*/train.py"))

    @classmethod
    def main(cls, argv: Sequence[str] | None = None) -> int:
        argv = list(sys.argv[1:] if argv is None else argv)
        if cls is Train:   # python -m training <task> ... → task의 train.main(argv)
            tasks = cls.tasks()
            if not argv or argv[0] in ("-h", "--help", "help"):
                print("사용법: python -m training <task> [옵션...]\n"
                      "  task: " + " | ".join(tasks))
                return 0 if argv else 2
            if argv[0] not in tasks:
                print(f"task 없음: {argv[0]} (있는 것: {', '.join(tasks)})", file=sys.stderr)
                return 2
            return importlib.import_module(f"training.{argv[0]}.train").main(argv[1:]) or 0

        if "--watch" in argv:
            argv.remove("--watch")
            if "--visualizer" not in argv and "--viz" not in argv:
                argv += ["--visualizer", "kit"]

        from isaaclab.app import AppLauncher

        parser = argparse.ArgumentParser(description=(cls.__doc__ or "").strip())
        parser.add_argument("--exp-name", default=cls.NAME, help="결과 폴더 이름 (<시각>-<exp-name>)")
        parser.add_argument("--snap", type=float, default=0.0, metavar="SEC",
                            help="SEC초마다 화면 저장 → 결과 폴더 snaps/ + latest.png (headless도 됨, 기본 0 = 끔)")
        cls.add_args(parser)
        AppLauncher.add_app_launcher_args(parser)
        args = parser.parse_args(argv)
        if args.snap and "kit" not in (args.visualizer or []):
            args.enable_cameras = True   # 창 없이 캡처하려면 렌더(replicator) 확장이 필요 — AppLauncher가 이걸로 켬
        cls.before_launch(args, parser)

        app_launcher = AppLauncher(args)
        try:
            train = cls(args)
            if train.watch:
                train.bind_screenshot_key()
            train.run()
        except BaseException:
            import traceback
            traceback.print_exc()   # app.close()가 프로세스를 바로 끝내서 예외가 안 보이고 종료코드도 0이 됨 → 먼저 출력
            raise
        finally:
            app_launcher.app.close()
        return 0
