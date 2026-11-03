"""URDF(xacro) → USD 변환 (로봇 공용). Isaac 앱이 켜진 뒤 호출."""
from __future__ import annotations

import os

from robots import ROBOTS_DIR


EXTERNAL_USD = os.path.join("usda", "robot.usda")   # 외부에서 받은 USD (task 폴더 등) — 있으면 변환 안 하고 그대로


def _dirs(robot: str):
    robot_dir = os.path.abspath(robot if os.sep in robot else os.path.join(ROBOTS_DIR, robot))   # 이름 또는 폴더 경로
    name = os.path.basename(robot_dir)
    external = os.path.join(robot_dir, EXTERNAL_USD)
    return robot_dir, name, external if os.path.isfile(external) else os.path.join(robot_dir, "usd", name, f"{name}.usda")


def usd_path(robot: str) -> str:
    """변환될 USD 경로만 (변환 안 함 · 표준 라이브러리). cfg는 Isaac 앱 전에 만들어질 수 있어서(공식 학습 스크립트) cfg엔 이걸 쓰고,
    실제 변환은 env가 장면을 만들 때 robot_usd()로 한다."""
    return _dirs(robot)[2]


def robot_usd(robot: str) -> str:
    """robots/<robot>/urdf/robot.urdf.xacro → robots/<robot>/usd/<robot>/<robot>.usda (Isaac 앱이 켜진 뒤 호출).

    URDF · meshes · 이 파일(변환 옵션)이 USD보다 새것일 때만 다시 변환 — URDF를 고치면 다음 실행에 반영.
    파일 잠금: Isaac 프로세스 여러 개(학습 + run-world 등)가 동시에 켜져도 서로의 변환물을 지우지 않게.
    usd/는 git 제외 (URDF가 정본, USD는 파생물). 메시는 robots/<robot>/meshes — xacro 안에서 `$(arg mesh_uri)`로 받는다.
    새 로봇: robots/<이름>/ 폴더에 urdf/robot.urdf.xacro + meshes/ + robot_cfg.py (숫자 · ROBOT_CFG).
    usda/robot.usda가 있는 폴더(training/<task>/)는 변환 없이 그 경로를 돌려준다.
    """
    import fcntl
    import glob
    import shutil

    robot_dir, robot, usd_path = _dirs(robot)
    if usd_path.endswith(EXTERNAL_USD):   # 외부 USD — 변환할 것 없음
        return usd_path
    xacro_path, out_dir = os.path.join(robot_dir, "urdf", "robot.urdf.xacro"), os.path.join(robot_dir, "usd")
    if not os.path.isfile(xacro_path):
        raise FileNotFoundError(f"로봇 URDF 없음: {xacro_path}")
    os.makedirs(out_dir, exist_ok=True)
    sources = [xacro_path, __file__] + glob.glob(os.path.join(robot_dir, "meshes", "**", "*"), recursive=True)
    with open(os.path.join(out_dir, ".lock"), "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)   # 다른 프로세스가 변환 중이면 끝날 때까지 기다림
        if os.path.isfile(usd_path) and os.path.getmtime(usd_path) > max(os.path.getmtime(f) for f in sources):
            return usd_path
        import xacro
        from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

        # importer는 같은 이름 폴더가 있으면 <name>_1, _2...로 쌓는다 — 이전 변환물 먼저 제거
        for old in glob.glob(os.path.join(out_dir, f"{robot}*")):
            if os.path.isdir(old):
                shutil.rmtree(old)
        urdf = os.path.join(out_dir, f"{robot}.urdf")
        with open(urdf, "w") as f:
            f.write(xacro.process_file(xacro_path, mappings={"mesh_uri": os.path.join(robot_dir, "meshes")})
                    .toprettyxml(indent="  "))
        cfg = UrdfConverterCfg(
            asset_path=urdf, usd_dir=out_dir, usd_file_name=f"{robot}.usd",
            force_usd_conversion=True, fix_base=False, merge_fixed_joints=False, joint_drive=None,
        )
        return UrdfConverter(cfg).usd_path
