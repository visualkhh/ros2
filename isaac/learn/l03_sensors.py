"""L3 — 센서: IMU + 2D LiDAR (RayCaster).

로봇에 실제와 같은 위치로 센서를 붙인다.
  IMU   : base_link 기준 (0.0178, 0, 0.0485) = imu_link
  LiDAR : base_link 기준 (0.060, 0, 0.012) = base_scan · 360° · 1채널 · 최대 3.5 m

실행:
  cd /home/pank/workspaces/visualkhh/source/ros2/isaac
  ./learn/run.sh l03_sensors.py

키:
  3/4/5/0  L1과 같음 (왼쪽/오른쪽/양쪽/정지)
  2        8방향 거리 + IMU 상세 출력
  1        센서 설정(prim path·offset·ray 수) 출력

화면: 벽·장애물에 빨간 점 = 라이다 hit. 로봇을 돌리면 점은 벽에 그대로 있고
      스캔 패턴만 회전한다.
콘솔: 1초마다 IMU 각속도·가속도와 라이다 앞/뒤/좌/우 거리.
"""
import argparse
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ISAAC = HERE.parent
sys.path.insert(0, str(ISAAC / "steps"))
sys.path.insert(0, str(ISAAC))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description="L3: IMU + LiDAR")
parser.add_argument("--room", type=float, default=4.0, help="방 한 변 [m]")
parser.add_argument("--seconds", type=float, default=0.0, help="0 = 창을 닫을 때까지")
parser.add_argument("--no-imu", action="store_true", help="IMU 없이 (디버그)")
parser.add_argument("--no-lidar", action="store_true", help="LiDAR 없이 (디버그)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import carb  # noqa: E402

try:
    import carb.input  # noqa: F401
    HAS_INPUT = True
except Exception:
    HAS_INPUT = False

import omni  # noqa: E402
import torch  # noqa: E402

from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.sensors import ImuCfg  # noqa: E402
from isaaclab.sensors.ray_caster import MultiMeshRayCasterCfg, patterns  # noqa: E402
from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402

from scene import bind_robot_materials, build_world, robot_articulation_cfg, set_camera, settle  # noqa: E402
from tasks.common import robot_spec as spec  # noqa: E402
from training.rl_correction_wheel_drift.task.rl_correct_env import wheel_targets  # noqa: E402

DT = 1 / 120.0
FRAME_DT = 1 / 60.0
LIDAR_MAX = 3.5
LIDAR_POS = (0.060, 0.0, 0.012)      # base_link 기준 (base_scan)
IMU_POS = (0.0178, 0.0, 0.0485)      # base_link 기준 (imu_link)


class Keyboard:
    DOWN = 1 << 3

    def __init__(self):
        self.MAP = {d: getattr(carb.input.KeyboardInput, f"KEY_{d}") for d in "012345"}
        self._input = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()

    @property
    def pressed(self) -> set[str]:
        flags = self._input.get_keyboard_button_flags
        return {name for name, key in self.MAP.items() if flags(self._keyboard, key) & self.DOWN}


def ray_index(deg: float, n: int) -> int:
    """패턴은 -180°부터 +180°까지 1° 간격. 0°(앞) = index n//2."""
    return int(round((deg + 180.0) % 360.0)) % n


def ranges_of(lidar) -> torch.Tensor:
    hits = lidar.data.ray_hits_w.torch[0]          # (N, 3)
    pos = lidar.data.pos_w.torch[0]                # (3,)
    r = torch.linalg.norm(hits - pos, dim=-1)
    return torch.nan_to_num(r, posinf=LIDAR_MAX, neginf=LIDAR_MAX)


def print_state(robot, imu, lidar, sim_t):
    if imu is not None:
        w_imu = imu.data.ang_vel_b.torch[0].tolist()
        a_imu = imu.data.lin_acc_b.torch[0].tolist()
        w_true = robot.data.root_ang_vel_b.torch[0].tolist()
        print(f"[l3] t={sim_t:5.1f}s  IMU w=({w_imu[0]:+.2f},{w_imu[1]:+.2f},{w_imu[2]:+.2f}) "
              f"true w_z={w_true[2]:+.2f} rad/s")
        print(f"     acc=({a_imu[0]:+.2f},{a_imu[1]:+.2f},{a_imu[2]:+.2f}) m/s²")
    if lidar is not None:
        rr = ranges_of(lidar)
        n = rr.numel()
        fwd, left, back, right = (rr[ray_index(0, n)], rr[ray_index(90, n)],
                                  rr[ray_index(180, n)], rr[ray_index(-90, n)])
        print(f"     LiDAR front={fwd:.2f} left={left:.2f} back={back:.2f} right={right:.2f} m "
              f"| min={rr.min():.2f} mean={rr.mean():.2f}")
        if n >= 360:
            dirs = {0: "앞", 45: "좌앞", 90: "왼쪽", 135: "좌뒤", 180: "뒤", 225: "우뒤", 270: "오른쪽", 315: "우앞"}
            print("     8방향: " + " · ".join(f"{name} {rr[ray_index(deg, n)]:.2f}" for deg, name in dirs.items()))


def main():
    headless = not getattr(args_cli, "visualizer", None) or "kit" not in (args_cli.visualizer or [])
    sim = SimulationContext(SimulationCfg(dt=DT, device=args_cli.device))
    set_camera(sim)
    build_world(1 if args_cli.room > 0 else 0, args_cli.room, 0)
    robot_cfg, usd_path = robot_articulation_cfg(prim_path="{ENV_REGEX_NS}/Robot")
    print(f"[l3] robot USD -> {usd_path}", flush=True)

    lidar_targets = [
        MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr="/World/ground", track_mesh_transforms=False),
    ]
    if args_cli.room > 0:
        lidar_targets.append(
            MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr="/World/rooms/room_0", track_mesh_transforms=False)
        )

    @configclass
    class L3SceneCfg(InteractiveSceneCfg):
        num_envs = 1
        env_spacing = 2.0
        robot = robot_cfg
        if not args_cli.no_imu:
            imu = ImuCfg(
                # imu_link는 base_link의 fixed child — 센서가 rigid 조상(base_link)을 찾아
                # fixed joint 변환을 자동 합성한다. offset을 또 주면 두 번 들어감.
                prim_path="{ENV_REGEX_NS}/Robot/Geometry/base_footprint/base_link/imu_link",
            )
        if not args_cli.no_lidar:
            lidar = MultiMeshRayCasterCfg(
                prim_path="{ENV_REGEX_NS}/Robot/Geometry/base_footprint/base_link",
                offset=MultiMeshRayCasterCfg.OffsetCfg(pos=LIDAR_POS),
                pattern_cfg=patterns.LidarPatternCfg(
                    channels=1,
                    vertical_fov_range=(0.0, 0.0),
                    horizontal_fov_range=(-180.0, 180.0),
                    horizontal_res=1.0,
                ),
                max_distance=LIDAR_MAX,
                mesh_prim_paths=lidar_targets,
                debug_vis=not headless,
            )

    scene = InteractiveScene(L3SceneCfg())
    robot = scene["robot"]
    imu = scene.sensors.get("imu")
    lidar = scene.sensors.get("lidar")
    bind_robot_materials(scene, prim_path="/World/envs/env_0/Robot")
    print("[l3] scene + sensors created", flush=True)
    sim.reset()
    print("[l3] sim.reset done", flush=True)
    settle(scene, robot, sim)
    print("[l3] settle done", flush=True)
    wheel_ids, _ = robot.find_joints(["wheel_left_joint", "wheel_right_joint"], preserve_order=True)
    dev = robot.device
    r = torch.tensor([spec.WHEEL_RADIUS], device=dev)
    sep = torch.tensor([spec.WHEEL_SEP], device=dev)
    print("[l3] IMU + 360° LiDAR 준비")
    print(f"[l3] IMU  prim=.../base_link/imu_link (URDF fixed joint {IMU_POS})")
    if lidar is not None:
        print(f"[l3] LiDAR prim=.../Geometry/base_footprint/base_link offset={LIDAR_POS} rays={lidar.num_rays} max={LIDAR_MAX} m")
    print("[l3] 3/4/5/0 = 바퀴 · 2 = 상세 · 1 = 센서 설정")
    print_state(robot, imu, lidar, 0.0)

    kb = Keyboard() if (HAS_INPUT and not headless) else None
    target = (0.0, 0.0)
    action_end = 0.0
    prev_keys = set()
    sim_t = 0.0
    last_print = 0.0
    t0 = time.perf_counter()
    while True:
        frame_start = time.perf_counter()
        wall = frame_start - t0
        if args_cli.seconds > 0 and wall >= args_cli.seconds:
            break
        if args_cli.seconds == 0 and not simulation_app.is_running():
            break

        keys = kb.pressed if kb else set()
        new = keys - prev_keys
        prev_keys = keys
        if "1" in new:
            rays = lidar.num_rays if lidar is not None else 0
            print(f"[l3] IMU prim=imu_link · LiDAR offset={LIDAR_POS} rays={rays} max={LIDAR_MAX}")
        if "2" in new:
            print_state(robot, imu, lidar, sim_t)
        if "3" in new:
            target, action_end = (4.0, 0.0), sim_t + 1.0
            print("[l3] 왼쪽 바퀴 +4 rad/s")
        if "4" in new:
            target, action_end = (0.0, 4.0), sim_t + 1.0
            print("[l3] 오른쪽 바퀴 +4 rad/s")
        if "5" in new:
            target, action_end = (4.0, 4.0), sim_t + 1.0
            print("[l3] 양쪽 +4 rad/s")
        if "0" in new:
            target, action_end = (0.0, 0.0), 0.0
            print("[l3] 정지")
        if target != (0.0, 0.0) and sim_t >= action_end:
            target = (0.0, 0.0)

        robot.set_joint_velocity_target(torch.tensor([target], device=dev), joint_ids=wheel_ids)
        scene.write_data_to_sim()
        for _ in range(2):
            sim.step(render=False)
        scene.update(2 * DT)
        if imu is not None:
            imu.update(2 * DT)
        if lidar is not None:
            lidar.update(2 * DT)
        sim.render()
        sim_t += 2 * DT

        if wall - last_print >= 1.0:
            last_print = wall
            print_state(robot, imu, lidar, sim_t)
        rest = FRAME_DT - (time.perf_counter() - frame_start)
        if rest > 0:
            time.sleep(rest)

    sim.stop()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
