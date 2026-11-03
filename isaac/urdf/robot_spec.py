"""우리 로봇(TurtleBot3 Waffle Pi 개조) 값 — 학습 환경이 쓰는 한 곳.

로봇 모양은 isaac/urdf/<robot>/urdf.xacro (우리 로봇 beebot) (ROS 쪽과 같은 내용). 수치는 ROS 쪽과 같아야 한다: ros2_ws/src/tb3_bringup/config/robot.yaml (바퀴) · opencr_driver.wheel_speeds (8 rad/s 비례 축소)
· cmd_manager 20 Hz · rl_correction corrector (max_lin/max_ang). 바꾸면 양쪽 같이.
"""
import os

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # ros2/ (이 파일 = isaac/urdf/)
# 로봇마다 폴더: urdf/<robot>/{urdf.xacro, meshes/, usd/(변환물, git 제외)}. 우리 로봇 = beebot
# (ROS용 tb3_bringup/urdf/robot.urdf.xacro와 의도적으로 나눈 파일, 내용은 같음 — isaac/urdf/check_same.sh)
ROBOTS_DIR = WS + "/isaac/urdf"
ROBOT = "beebot"

WHEEL_RADIUS = 0.033   # [m]
WHEEL_SEP = 0.287      # [m] (드라이버 기준값. URDF 바퀴 y ±0.144)
MAX_LIN = 0.26         # [m/s]
MAX_ANG = 1.82         # [rad/s]
MAX_WHEEL = 8.0        # [rad/s] 펌웨어 VEL_LIMIT_RAD_S · 드라이버 max_wheel_speed
CONTROL_HZ = 20.0      # cmd_manager mux 주기 = 정책 주기
# 물리 계산 주기. 120/240 Hz는 제자리 회전 때 바퀴가 바닥에 붙었다 미끄러졌다 해서 바퀴 속도가 떨림
# (정상 회전 구간 표준편차 120 Hz 0.62 · 240 Hz 0.15 · 360 Hz 0.11 · 480 Hz 0.027 · real 0.018 rad/s, 2026-11-02 실측 비교)
PHYSICS_HZ = 480.0
DIAG_RADIUS = 0.5      # [m] 텔레오퍼 사선 원호 (tool/cli/op/bus.py DIAG_RADIUS)


def robot_usd(robot: str = ROBOT) -> str:
    """urdf/<robot>/urdf.xacro → urdf/<robot>/usd/<robot>.usd (Isaac 앱이 켜진 뒤 호출). 매번 새로 변환 — URDF를 고치면 바로 반영.

    usd/는 git 제외 (URDF가 정본, USD는 파생물). 메시는 urdf/<robot>/meshes — xacro 안에서 `$(arg mesh_uri)`로 받는다.
    새 로봇: urdf/<이름>/ 폴더에 urdf.xacro + meshes/ 를 두고 robot_usd("<이름>").
    """
    import glob
    import shutil

    import xacro
    from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

    robot_dir = os.path.join(ROBOTS_DIR, robot)
    xacro_path, out_dir = os.path.join(robot_dir, "urdf.xacro"), os.path.join(robot_dir, "usd")
    if not os.path.isfile(xacro_path):
        raise FileNotFoundError(f"로봇 URDF 없음: {xacro_path}")
    os.makedirs(out_dir, exist_ok=True)
    # importer는 같은 이름 폴더가 있으면 <name>_1, _2...로 쌓는다 — 이전 변환물 먼저 제거
    for old in glob.glob(os.path.join(out_dir, f"{robot}*")):
        if os.path.isdir(old):
            shutil.rmtree(old)
    urdf = os.path.join(out_dir, f"{robot}.urdf")
    mesh_uri = os.path.join(robot_dir, "meshes")
    with open(urdf, "w") as f:
        f.write(xacro.process_file(xacro_path, mappings={"mesh_uri": mesh_uri}).toprettyxml(indent="  "))
    cfg = UrdfConverterCfg(
        asset_path=urdf, usd_dir=out_dir, usd_file_name=f"{robot}.usd",
        force_usd_conversion=True, fix_base=False, merge_fixed_joints=False, joint_drive=None,
    )
    return UrdfConverter(cfg).usd_path
