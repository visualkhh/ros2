"""calibration env — 로봇 1대에 [v, w] 명령을 넣고 엔코더 · 자이로를 읽는다 (Isaac Lab Direct 관례).

한 스텝(20 Hz): 명령 → [지연] → 바퀴 rad/s (드라이버 식) → 좌/우 gain → 데드밴드 → 1차 지연 τ → 바퀴 속도 목표 → 물리 24회
관측 = 드라이버가 내는 값: v_enc · w_enc (/odom, 기준 반지름·간격) · w_gyro (/imu = 몸체 각속도).
학습이 아니라 측정용이라 보상 0 · 종료 없음. 기록·명령 경로는 스크립트(sensor_sync_sim · random_world)가 한다.
"""
from __future__ import annotations

import math
import random
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

import isaaclab.sim as sim_utils
from isaaclab import cloner
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.utils.math import quat_from_euler_xyz

from assets.tb3_waffle_pi import bind_wheel_materials, wheel_targets
from urdf import robot_spec as spec

if TYPE_CHECKING:
    from calibration.calib_env_cfg import CalibEnvCfg

ROBOT_Z = 0.03          # 스폰 높이 [m] — 떨어져서 바퀴·캐스터로 선다
FREE_RADIUS = 0.75      # 방 가운데 로봇 자리 — 장애물 금지
WALL_H, WALL_T = 1.1, 0.05
PALETTE = ((0.75, 0.15, 0.12), (0.12, 0.55, 0.20), (0.15, 0.30, 0.75),
           (0.75, 0.55, 0.10), (0.45, 0.20, 0.60), (0.20, 0.60, 0.65))


class CalibEnv(DirectRLEnv):
    cfg: CalibEnvCfg

    def __init__(self, cfg: CalibEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._wheels, _ = self.robot.find_joints(["wheel_left_joint", "wheel_right_joint"], preserve_order=True)
        dev = self.device
        self.policy_dt = self.cfg.sim.dt * self.cfg.decimation
        self.r = torch.tensor([spec.WHEEL_RADIUS], device=dev)
        self.sep = torch.tensor([spec.WHEEL_SEP], device=dev)
        self.cmd = torch.zeros(1, 2, device=dev)
        self.target = torch.zeros(1, 2, device=dev)   # 바퀴 속도 목표 [rad/s] (바퀴 모델 출력)
        self.path_w = None                            # 목표 경로 점 (m, 3) — 스크립트가 넣음 (화면용)
        self.reset_wheel_model()

    # ------------------------------------------------------------------ 장면
    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot_cfg)
        self.scene.articulations["robot"] = self.robot
        bind_wheel_materials(self.scene.stage, "/World/envs/env_0/Robot")
        self.cfg.terrain.num_envs = self.scene.cfg.num_envs
        self.cfg.terrain.env_spacing = self.scene.cfg.env_spacing
        self._terrain = self.cfg.terrain.class_type(self.cfg.terrain)
        pos = cloner.grid_transforms(self.scene.num_envs, self.scene.cfg.env_spacing, device=self.device)[0]
        plan = cloner.clone_plan_from_env_0("/World/envs/env_0", "/World/envs/env_{}", self.scene.num_envs,
                                            self.device, pos, global_paths=(self.cfg.terrain.prim_path,))
        cloner.replicate(plan, stage=self.scene.stage)
        light = sim_utils.DistantLightCfg(intensity=3000.0, color=(0.95, 0.95, 0.95))
        light.func("/World/light", light, translation=(2.0, -2.0, 4.0))
        self.rooms = spawn_rooms(self.cfg)

    # ------------------------------------------------------------------ 바퀴 모델
    def reset_wheel_model(self):
        self._buf = torch.zeros(int(self.cfg.wheel.delay_steps) + 1, 2, device=self.device)   # [0] = 최신 명령
        self._lag = None

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        w = self.cfg.wheel   # 실행 중 바뀔 수 있다 (random_world 보정값 창)
        self.cmd = actions[:1].clone()
        n = max(0, int(w.delay_steps)) + 1
        if self._buf.shape[0] != n:   # 지연이 바뀌면 마지막 명령으로 다시 채운다
            self._buf = self._buf[:1].expand(n, 2).clone()
        self._buf = torch.roll(self._buf, 1, dims=0)
        self._buf[0] = self.cmd[0]
        v, wz = self._buf[-1]
        wheels = wheel_targets(v[None], wz[None], self.r, self.sep, spec.MAX_WHEEL)
        wheels = wheels * wheels.new_tensor([w.gain_l, w.gain_r])
        if w.deadband > 0:
            wheels = torch.where(wheels.abs() < w.deadband, torch.zeros_like(wheels), wheels)
        if w.tau_s <= 1e-6 or self._lag is None:
            self._lag = wheels
        else:
            self._lag = self._lag + (1.0 - math.exp(-self.policy_dt / w.tau_s)) * (wheels - self._lag)
        self.target = self._lag

    def _apply_action(self) -> None:
        self.robot.set_joint_velocity_target(self.target, joint_ids=self._wheels)

    # ------------------------------------------------------------------ 관측
    def sense(self) -> dict:
        """드라이버와 같은 식 — 바퀴 rad/s · 각도, /odom v·w (기준 반지름·간격), /imu w (몸체 각속도)."""
        d = self.robot.data
        jv = d.joint_vel.torch[0, self._wheels].tolist()
        jp = d.joint_pos.torch[0, self._wheels].tolist()
        return {"jv": jv, "jp": jp,
                "v_enc": spec.WHEEL_RADIUS * (jv[0] + jv[1]) / 2.0,
                "w_enc": spec.WHEEL_RADIUS * (jv[1] - jv[0]) / spec.WHEEL_SEP,
                "w_gyro": d.root_ang_vel_b.torch[0, 2].item()}

    def _get_observations(self) -> dict:
        s = self.sense()
        return {"policy": torch.tensor([[s["v_enc"], s["w_enc"], s["w_gyro"]]], device=self.device)}

    def _get_rewards(self) -> torch.Tensor:
        return torch.zeros(self.num_envs, device=self.device)

    def _get_dones(self):
        no = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        return no, no

    # ------------------------------------------------------------------ 리셋 · 이동
    def _reset_idx(self, env_ids: Sequence[int] | None):
        super()._reset_idx(env_ids)
        self.place((0.0, 0.0, 0.0))

    def place(self, origin, yaw: float = 0.0):
        """로봇을 origin(월드)으로 순간이동 — 속도 · 바퀴 상태 · 바퀴 모델 0."""
        dev = self.device
        root = self.robot.data.default_root_state.torch[:1].clone()
        root[:, 0], root[:, 1], root[:, 2] = origin[0], origin[1], ROBOT_Z
        zero = torch.zeros(1, device=dev)
        root[:, 3:7] = quat_from_euler_xyz(zero, zero, torch.tensor([yaw], device=dev))
        self.robot.write_root_pose_to_sim(root[:, :7])
        self.robot.write_root_velocity_to_sim(torch.zeros(1, 6, device=dev))
        z = torch.zeros(1, len(self._wheels), device=dev) if hasattr(self, "_wheels") else None
        if z is not None:
            self.robot.write_joint_state_to_sim(z, z, self._wheels)
            self.reset_wheel_model()
            self.target = torch.zeros(1, 2, device=dev)

    def look_at(self, origin):
        self.sim.set_camera_view(eye=(origin[0] + 1.8, origin[1] - 1.8, 1.55), target=(origin[0], origin[1], 0.15))

    # ------------------------------------------------------------------ 화면 (debug_vis)
    def _set_debug_vis_impl(self, debug_vis: bool):
        if debug_vis and getattr(self, "markers", None) is None:
            from calibration.command_markers import CommandMarkers
            self.markers = CommandMarkers()
        if getattr(self, "markers", None) is not None:
            self.markers.set_visible(debug_vis)

    def _debug_vis_callback(self, event):
        if not hasattr(self, "cmd"):   # 부모 __init__ 도중
            return
        d = self.robot.data
        yaw = self.markers.yaw_of(d.root_quat_w.torch)
        self.markers.update(d.root_pos_w.torch, yaw, self.cmd[:, 0], self.cmd[:, 1],
                            d.root_lin_vel_b.torch[:, 0], d.root_ang_vel_b.torch[:, 2], self.path_w)


# ---------------------------------------------------------------------- 방 + 장애물
def spawn_rooms(cfg) -> list[dict]:
    """cfg.rooms개 방 (벽 4 + 장애물 3~6개, 방마다 크기 랜덤). 방 i 가운데 = (i × room_spacing, 0). 반환: [{origin, size, obstacles}]."""
    if cfg.rooms <= 0:
        return []
    mat = sim_utils.RigidBodyMaterialCfg(static_friction=cfg.obstacle_friction[0],
                                         dynamic_friction=cfg.obstacle_friction[1],
                                         restitution=0.0, friction_combine_mode="average")
    mat.func("/World/Materials/obstacle", mat)
    rooms = []
    for i in range(cfg.rooms):
        rng = random.Random(cfg.room_seed + i * 1000)
        size = random.Random(cfg.room_seed + i * 7919).uniform(*cfg.room_size)
        origin = (i * cfg.room_spacing, 0.0, 0.0)
        half, ox, tag = size / 2.0, origin[0], f"/World/rooms/room_{i}"
        wall = (0.72, 0.72, 0.70)
        for name, dims, pos in (("px", (WALL_T, size, WALL_H), (ox + half, 0.0)), ("nx", (WALL_T, size, WALL_H), (ox - half, 0.0)),
                                ("py", (size, WALL_T, WALL_H), (ox, half)), ("ny", (size, WALL_T, WALL_H), (ox, -half))):
            _box(f"{tag}/wall_{name}", dims, (*pos, WALL_H / 2), wall)
        obstacles = _sample_obstacles(rng, half, rng.randint(3, 6))
        for j, (kind, dims, x, y) in enumerate(obstacles):
            color = PALETTE[(i * 3 + j) % len(PALETTE)]
            path = f"{tag}/obstacle_{j}"
            if kind == "box":
                _box(path, dims, (ox + x, y, dims[2] / 2), color, yaw=rng.uniform(-math.pi, math.pi))
            else:
                c = sim_utils.CylinderCfg(radius=dims[0], height=dims[1], collision_props=sim_utils.PhysxCollisionPropertiesCfg(),
                                          visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, roughness=0.6))
                c.func(path, c, translation=(ox + x, y, dims[1] / 2))
            sim_utils.bind_physics_material(f"{path}/geometry/mesh", "/World/Materials/obstacle")
        rooms.append({"origin": origin, "size": size, "obstacles": len(obstacles)})
    return rooms


def _box(path, size, pos, color, yaw: float = 0.0):
    cfg = sim_utils.CuboidCfg(size=size, collision_props=sim_utils.PhysxCollisionPropertiesCfg(),
                              visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, roughness=0.6))
    quat = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)) if yaw else None
    cfg.func(path, cfg, translation=pos, orientation=quat)


def _sample_obstacles(rng: random.Random, half: float, count: int):
    """겹치지 않게 배치, 로봇 자리(가운데 FREE_RADIUS)는 비움. [(kind, dims, x, y)]."""
    placed, out = [], []
    for _ in range(count):
        for _try in range(200):
            if rng.random() < 0.72:
                w, d, h = rng.uniform(0.20, 0.50), rng.uniform(0.20, 0.50), rng.uniform(0.18, 0.80)
                rad, kind, dims = 0.5 * math.hypot(w, d), "box", (w, d, h)
            else:
                rad, h = rng.uniform(0.10, 0.22), rng.uniform(0.30, 0.80)
                kind, dims = "cyl", (rad, h)
            x, y = rng.uniform(-half + 0.35, half - 0.35), rng.uniform(-half + 0.35, half - 0.35)
            if math.hypot(x, y) < FREE_RADIUS + rad or any(math.hypot(x - px, y - py) < rad + pr + 0.25 for px, py, pr in placed):
                continue
            placed.append((x, y, rad))
            out.append((kind, dims, x, y))
            break
    return out
