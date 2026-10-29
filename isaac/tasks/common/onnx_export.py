"""학습된 actor → ONNX (train.py 끝과 export.py가 같이 씀).

입력 `obs` [1, N] float32 → 출력 `actions` [1, 2] = 물리 보정량 [dV m/s, dW rad/s]
(env와 같은 clamp(-1, 1) × action_scale을 그래프에 넣음 → ROS corrector는 더하기만).
"""
import os

import torch


class _Actor(torch.nn.Module):
    def __init__(self, actor, action_scale):
        super().__init__()
        self.norm, self.mlp, self.dist = actor.obs_normalizer, actor.mlp, actor.distribution  # 복사 없이 참조 (읽기만)
        self.register_buffer("scale", torch.tensor(action_scale, dtype=torch.float32))

    def forward(self, x):
        return self.dist.deterministic_output(self.mlp(self.norm(x))).clamp(-1.0, 1.0) * self.scale


def export_actor(actor, action_scale, path: str, device: str = "cuda") -> int:
    """ONNX 저장, obs 차원 반환."""
    actor.eval()
    obs_dim = actor.mlp[0].in_features
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with torch.no_grad():
        torch.onnx.export(_Actor(actor, action_scale).eval().to(device), torch.zeros(1, obs_dim, device=device), path,
                          input_names=["obs"], output_names=["actions"], opset_version=18, dynamo=False)
    return obs_dim
