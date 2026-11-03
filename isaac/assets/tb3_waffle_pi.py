"""TurtleBot3 Waffle Pi (우리 개조판: 상판 제거 · LDS-03 1층 앞) — 로봇 정의 한 곳.

Isaac Lab `isaaclab_assets/robots/anymal.py`의 `ANYMAL_C_CFG`와 같은 관례:
학습 env · calibration env가 `TB3_CFG.replace(prim_path=...)`로 같은 로봇을 쓴다.
USD는 URDF(xacro)에서 그때그때 변환 — `robot_usd()`가 Isaac 앱을 요구하므로 AppLauncher 뒤에 import한다.
"""
from __future__ import annotations

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.sim.spawners.from_files import UsdFileCfg

from urdf import robot_spec as spec

# 바퀴 모터 — 고정 모델값 (실측 sweep해도 sim 응답이 안 변해서 calibration 변수가 아님)
WHEEL_DAMPING = 50.0         # 속도 서보 이득 (ImplicitActuator, stiffness 0)
WHEEL_EFFORT = 3.0           # [N·m] XM430-W210 stall 근사
WHEEL_VELOCITY_LIMIT = 9.0   # [rad/s] MAX_WHEEL(8) + 여유 — 포화는 env 바퀴 모델이 드라이버처럼 비례 축소로 직접

# 마찰 (static, dynamic) — 바퀴 · 캐스터는 env가 재질로 붙이고, 바닥 · 장애물은 env의 지형/장면이 쓴다
GROUND_FRICTION = (0.8, 0.7)
OBSTACLE_FRICTION = (0.7, 0.5)
WHEEL_FRICTION = (1.0, 1.0)
CASTER_FRICTION = (0.0, 0.0)   # 캐스터는 min 결합 — average면 바닥 마찰에 끌려 바퀴가 덜 나감

TB3_CFG = ArticulationCfg(
    prim_path="/World/Robot",
    spawn=UsdFileCfg(usd_path=spec.robot_usd()),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.01)),
    actuators={
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=["wheel_left_joint", "wheel_right_joint"],
            effort_limit=WHEEL_EFFORT, velocity_limit=WHEEL_VELOCITY_LIMIT,
            stiffness=0.0, damping=WHEEL_DAMPING),
    },
)


def wheel_targets(v, w, r, sep, max_wheel):
    """Twist → (좌, 우) 바퀴 rad/s. 한 바퀴라도 max_wheel을 넘으면 두 바퀴를 같은 비율로 (= opencr_driver.wheel_speeds)."""
    import torch

    wl = (v - w * sep / 2.0) / r
    wr = (v + w * sep / 2.0) / r
    wheels = torch.stack([wl, wr], dim=-1)
    peak = wheels.abs().max(dim=-1, keepdim=True).values
    return wheels * torch.clamp(max_wheel / (peak + 1e-9), max=1.0)


def bind_wheel_materials(stage, robot_root: str, wheel=WHEEL_FRICTION, caster=CASTER_FRICTION) -> None:
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
