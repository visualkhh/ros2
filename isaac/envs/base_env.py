"""기본 세상 — 로봇 + 바닥을 만들고, [v, w] 명령을 드라이버 식으로 바퀴에 넣고, 드라이버가 내는 값을 읽는다 (Isaac Lab Direct).

공통으로 해 주는 것 (task env는 필요한 메서드만 덮어씀):
- `_setup_scene`: 로봇 → 바퀴/캐스터 재질 → 지형 → env 복제 → env끼리 충돌 끔 → 조명 (anymal_c_direct 순서)
- `wheel_speeds(v, w)`: Twist → 좌/우 rad/s (8 rad/s 넘으면 비례 축소 = opencr_driver)
- `encoders(r, sep)` · `true_twist()` · `sense(i)`: /odom v·w (엔코더) · 몸체 실제 속도 · env i 한 줄
- `_reset_idx` / `place`: 지형 칸 가운데에 세우기 (cfg.random_yaw면 방향 랜덤) · 순간이동
- 화면(debug_vis): 명령/실제 막대 + `path_w` 목표 경로
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

import isaaclab.sim as sim_utils
from isaaclab import cloner
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.utils.math import quat_from_euler_xyz, sample_uniform

if TYPE_CHECKING:
    from envs.base_env_cfg import BaseEnvCfg

ROBOT_Z = 0.03   # 순간이동 높이 [m] — 떨어져서 바퀴·캐스터로 선다


class BaseEnv(DirectRLEnv):
    cfg: BaseEnvCfg

    def __init__(self, cfg: BaseEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._wheels, _ = self.robot.find_joints(["wheel_left_joint", "wheel_right_joint"], preserve_order=True)
        n, dev = self.num_envs, self.device
        self.policy_dt = self.cfg.sim.dt * self.cfg.decimation
        self.r = torch.tensor([self.cfg.wheel_radius], device=dev)
        self.sep = torch.tensor([self.cfg.wheel_sep], device=dev)
        self.cmd = torch.zeros(n, 2, device=dev)       # 이번 스텝 명령 [v, w]
        self.target = torch.zeros(n, 2, device=dev)    # 바퀴 속도 목표 [rad/s]
        self.path_w = None                             # 목표 경로 점 (m, 3) — 화면용, 스크립트가 넣음

    # ------------------------------------------------------------------ 장면
    def _setup_scene(self):
        from robots.urdf_to_usd import robot_usd

        import robots

        robots.use(self.cfg, self.cfg.robot)   # 덮어쓰기(env.robot=…)가 반영되게 여기서 한 번 더
        robot_usd(self.cfg.robot)   # URDF → USD (바뀌었을 때만) — Isaac 앱이 뜬 지금 (cfg엔 경로만 있다)
        self.robot = Articulation(self.cfg.robot_cfg)
        self.scene.articulations["robot"] = self.robot
        bind_wheel_materials(self.scene.stage, "/World/envs/env_0/Robot", self.cfg.wheel_friction, self.cfg.caster_friction)
        self.cfg.terrain.num_envs = self.scene.cfg.num_envs
        self.cfg.terrain.env_spacing = self.scene.cfg.env_spacing
        self._terrain = self.cfg.terrain.class_type(self.cfg.terrain)
        pos = cloner.grid_transforms(self.scene.num_envs, self.scene.cfg.env_spacing, device=self.device)[0]
        plan = cloner.clone_plan_from_env_0("/World/envs/env_0", "/World/envs/env_{}", self.scene.num_envs,
                                            self.device, pos, global_paths=(self.cfg.terrain.prim_path,))
        cloner.replicate(plan, stage=self.scene.stage)
        if "physx" in self.scene.physics_backend:
            self.scene.filter_collisions(global_prim_paths=[self.cfg.terrain.prim_path])
        light = sim_utils.DistantLightCfg(intensity=3000.0, color=(0.95, 0.95, 0.95))
        light.func("/World/light", light, translation=(2.0, -2.0, 4.0))

    # ------------------------------------------------------------------ 로봇 식 (드라이버와 같음)
    def wheel_speeds(self, v: torch.Tensor, w: torch.Tensor, r=None, sep=None) -> torch.Tensor:
        return wheel_targets(v, w, self.r if r is None else r, self.sep if sep is None else sep, self.cfg.max_wheel)

    def encoders(self, r=None, sep=None) -> tuple[torch.Tensor, torch.Tensor]:
        """/odom v · w = 바퀴 rad/s를 기준 반지름 · 간격으로 (드라이버 식). (n,) 둘."""
        jv = torch.nan_to_num(self.robot.data.joint_vel.torch[:, self._wheels])
        r = self.r if r is None else r
        sep = self.sep if sep is None else sep
        return r * (jv[:, 0] + jv[:, 1]) / 2.0, r * (jv[:, 1] - jv[:, 0]) / sep

    def true_twist(self) -> tuple[torch.Tensor, torch.Tensor]:
        """몸체 실제 전진 · 회전 속도 (몸체 기준, 경사에서도 맞음). w = /imu 자이로."""
        d = self.robot.data
        return torch.nan_to_num(d.root_lin_vel_b.torch[:, 0]), torch.nan_to_num(d.root_ang_vel_b.torch[:, 2])

    def sense(self, i: int = 0) -> dict:
        """env i 한 줄 — 바퀴 rad/s · 각도, v_enc · w_enc, w_gyro (기록용 float)."""
        d = self.robot.data
        v_enc, w_enc = self.encoders()
        return {"jv": d.joint_vel.torch[i, self._wheels].tolist(), "jp": d.joint_pos.torch[i, self._wheels].tolist(),
                "v_enc": float(v_enc[i]), "w_enc": float(w_enc[i]), "w_gyro": float(self.true_twist()[1][i])}

    # ------------------------------------------------------------------ 한 스텝 (기본: 명령 → 바퀴 그대로)
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.cmd = actions.clone()
        self.target = self.wheel_speeds(self.cmd[:, 0], self.cmd[:, 1])

    def _apply_action(self) -> None:
        self.robot.set_joint_velocity_target(self.target, joint_ids=self._wheels)

    def _get_observations(self) -> dict:
        v_enc, w_enc = self.encoders()
        return {"policy": torch.stack([v_enc, w_enc, self.true_twist()[1]], dim=-1)}

    def _get_rewards(self) -> torch.Tensor:
        return torch.zeros(self.num_envs, device=self.device)

    def _get_dones(self):
        time_out = self.episode_length_buf >= self.max_episode_length
        return torch.zeros_like(time_out), time_out

    # ------------------------------------------------------------------ 리셋 · 이동
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        ids = env_ids if isinstance(env_ids, torch.Tensor) else torch.as_tensor(env_ids, device=self.device)
        n, dev = len(ids), self.device
        root = self.robot.data.default_root_state.torch[ids].clone()
        root[:, :3] += self._terrain.env_origins[ids]   # 지형 칸 가운데 (경사 칸이면 높이 포함)
        yaw = sample_uniform(-math.pi, math.pi, (n,), dev) if self.cfg.random_yaw else torch.zeros(n, device=dev)
        root[:, 3:7] = quat_from_euler_xyz(torch.zeros_like(yaw), torch.zeros_like(yaw), yaw)
        self.robot.write_root_pose_to_sim(root[:, :7], ids)
        self.robot.write_root_velocity_to_sim(torch.zeros(n, 6, device=dev), ids)
        zeros = torch.zeros(n, len(self._wheels), device=dev)
        self.robot.write_joint_state_to_sim(zeros, zeros, self._wheels, ids)
        if not hasattr(self, "reset_pose"):   # 리셋 자세 (월드 x, y, yaw) — sim에서 다시 읽으면 다음 스텝 전엔 옛 값
            self.reset_pose = torch.zeros(self.num_envs, 3, device=dev)
        self.reset_pose[ids] = torch.stack([root[:, 0], root[:, 1], yaw], dim=-1)

    def place(self, origin, yaw: float = 0.0, i: int = 0) -> None:
        """env i 로봇을 월드 origin으로 순간이동 — 속도 · 바퀴 0."""
        dev, ids = self.device, torch.tensor([i], device=self.device)
        root = self.robot.data.default_root_state.torch[ids].clone()
        root[:, 0], root[:, 1], root[:, 2] = origin[0], origin[1], ROBOT_Z
        zero = torch.zeros(1, device=dev)
        root[:, 3:7] = quat_from_euler_xyz(zero, zero, torch.tensor([yaw], device=dev))
        self.robot.write_root_pose_to_sim(root[:, :7], ids)
        self.robot.write_root_velocity_to_sim(torch.zeros(1, 6, device=dev), ids)
        z = torch.zeros(1, len(self._wheels), device=dev)
        self.robot.write_joint_state_to_sim(z, z, self._wheels, ids)
        self.target[ids] = 0.0

    def set_path(self, xy, i: int = 0) -> None:
        """목표 경로(명령의 path(): 시작 자세 기준 [(x 앞, y 왼쪽)])를 env i 로봇의 지금 자세로 옮겨 노란 점으로. None이면 끔."""
        if not xy:
            self.path_w = None
            return
        d = self.robot.data
        p0, q = d.root_pos_w.torch[i].tolist(), d.root_quat_w.torch[i].tolist()   # xyzw
        yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
        pts = []
        for (x0, y0), (x1, y1) in zip(xy, xy[1:]):   # 3 cm 간격 점
            k = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.03))
            pts += [(x0 + (x1 - x0) * j / k, y0 + (y1 - y0) * j / k) for j in range(k)]
        c, s = math.cos(yaw), math.sin(yaw)
        self.path_w = torch.tensor([[p0[0] + c * x - s * y, p0[1] + s * x + c * y, p0[2]] for x, y in pts], device=self.device)

    def look_at(self, origin) -> None:
        self.sim.set_camera_view(eye=(origin[0] + 1.8, origin[1] - 1.8, 1.55), target=(origin[0], origin[1], 0.15))

    # ------------------------------------------------------------------ 화면 (debug_vis)
    def _set_debug_vis_impl(self, debug_vis: bool):
        if debug_vis and getattr(self, "markers", None) is None:
            from calibration.command_markers import CommandMarkers
            self.markers = CommandMarkers()
        if getattr(self, "markers", None) is not None:
            self.markers.set_visible(debug_vis)

    def vis_path(self):
        """노란 점 — 기본은 스크립트가 넣은 path_w. task가 덮어씀 (학습: 명령대로 갈 원호)."""
        return self.path_w

    def _debug_vis_callback(self, event):
        if not hasattr(self, "path_w"):   # 부모 __init__ 도중
            return
        d = self.robot.data
        v, w = self.true_twist()
        self.markers.update(d.root_pos_w.torch, self.markers.yaw_of(d.root_quat_w.torch),
                            self.cmd[:, 0], self.cmd[:, 1], v, w, self.vis_path())


def wheel_targets(v, w, r, sep, max_wheel):
    """Twist → (좌, 우) 바퀴 rad/s. 한 바퀴라도 max_wheel을 넘으면 두 바퀴를 같은 비율로 (= opencr_driver.wheel_speeds)."""
    import torch

    wl = (v - w * sep / 2.0) / r
    wr = (v + w * sep / 2.0) / r
    wheels = torch.stack([wl, wr], dim=-1)
    peak = wheels.abs().max(dim=-1, keepdim=True).values
    return wheels * torch.clamp(max_wheel / (peak + 1e-9), max=1.0)


def bind_wheel_materials(stage, robot_root: str, wheel, caster) -> None:
    """바퀴 · 캐스터 충돌체에 마찰 재질 (robot_root 아래 전부 — 복제 전 env_0에 붙이면 복제로 퍼진다).

    URDF importer는 generic RigidBodyAPI만 넣는다 → PhysX 센서 뷰가 요구하는 PhysxRigidBodyAPI도 붙인다.
    """
    import isaaclab.sim as sim_utils
    from pxr import PhysxSchema, UsdPhysics

    for name, fr, mode in (("wheel", wheel, "average"), ("caster", caster, "min")):
        mat = sim_utils.RigidBodyMaterialCfg(static_friction=fr[0], dynamic_friction=fr[1],
                                             restitution=0.0, friction_combine_mode=mode)
        mat.func(f"/World/Materials/{name}", mat)
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(robot_root):
            continue
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and not prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            for name in ("wheel", "caster"):
                if name in path:
                    sim_utils.bind_physics_material(path, f"/World/Materials/{name}")
                    break
