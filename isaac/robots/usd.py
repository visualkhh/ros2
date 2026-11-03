"""URDF(xacro) → USD 변환 (로봇 공용). Isaac 앱이 켜진 뒤 호출."""
from __future__ import annotations

import os

from robots import ROBOTS_DIR


def robot_usd(robot: str) -> str:
    """robots/<robot>/urdf.xacro → robots/<robot>/usd/<robot>/<robot>.usda (Isaac 앱이 켜진 뒤 호출).

    URDF · meshes · 이 파일(변환 옵션)이 USD보다 새것일 때만 다시 변환 — URDF를 고치면 다음 실행에 반영.
    파일 잠금: Isaac 프로세스 여러 개(학습 + run-world 등)가 동시에 켜져도 서로의 변환물을 지우지 않게.
    usd/는 git 제외 (URDF가 정본, USD는 파생물). 메시는 robots/<robot>/meshes — xacro 안에서 `$(arg mesh_uri)`로 받는다.
    새 로봇: robots/<이름>/ 폴더에 urdf.xacro + meshes/ + robot_cfg.py (ROBOT_CFG) + spec.py.
    """
    import fcntl
    import glob
    import shutil

    robot_dir = os.path.join(ROBOTS_DIR, robot)
    xacro_path, out_dir = os.path.join(robot_dir, "urdf.xacro"), os.path.join(robot_dir, "usd")
    if not os.path.isfile(xacro_path):
        raise FileNotFoundError(f"로봇 URDF 없음: {xacro_path}")
    os.makedirs(out_dir, exist_ok=True)
    usd_path = os.path.join(out_dir, robot, f"{robot}.usda")
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
