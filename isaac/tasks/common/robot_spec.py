"""우리 로봇(TurtleBot3 Waffle Pi 개조) 값 — 학습 환경이 쓰는 한 곳.

로봇 모양은 isaac/urdf/beebot/urdf.xacro (ROS 쪽과 같은 내용). 수치는 ROS 쪽과 같아야 한다: ros2_ws/src/tb3_bringup/config/robot.yaml (바퀴) · opencr_driver.wheel_speeds (8 rad/s 비례 축소)
· cmd_manager 20 Hz · rl_correction corrector (max_lin/max_ang). 바꾸면 양쪽 같이.
"""
import os

WS = "/home/pank/workspaces/visualkhh/source/ros2"
# 학습용 URDF (ROS용 tb3_bringup/urdf/robot.urdf.xacro와 의도적으로 나눈 파일, 내용은 같음 — isaac/urdf/check_same.sh)
URDF_DIR = WS + "/isaac/urdf/beebot"
XACRO = URDF_DIR + "/urdf.xacro"

WHEEL_RADIUS = 0.033   # [m]
WHEEL_SEP = 0.287      # [m] (드라이버 기준값. URDF 바퀴 y ±0.144)
MAX_LIN = 0.26         # [m/s]
MAX_ANG = 1.82         # [rad/s]
MAX_WHEEL = 8.0        # [rad/s] 펌웨어 VEL_LIMIT_RAD_S · 드라이버 max_wheel_speed
CONTROL_HZ = 20.0      # cmd_manager mux 주기 = 정책 주기
DIAG_RADIUS = 0.5      # [m] 텔레오퍼 사선 원호 (tool/cli/op/bus.py DIAG_RADIUS)


def robot_usd(out_dir: str = "/tmp/tb3_usd") -> str:
    """xacro → URDF → USD (Isaac 앱이 켜진 뒤 호출). 매번 새로 변환 — URDF를 고치면 바로 반영."""
    import xacro
    from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

    urdf = os.path.join(out_dir, "tb3_waffle_pi.urdf")
    os.makedirs(out_dir, exist_ok=True)
    with open(urdf, "w") as f:
        f.write(xacro.process_file(XACRO, mappings={"mesh_uri": URDF_DIR + "/meshes"}).toprettyxml(indent="  "))
    cfg = UrdfConverterCfg(
        asset_path=urdf, usd_dir=out_dir, usd_file_name="tb3_waffle_pi.usd",
        force_usd_conversion=True, fix_base=False, merge_fixed_joints=False, joint_drive=None,
    )
    return UrdfConverter(cfg).usd_path
