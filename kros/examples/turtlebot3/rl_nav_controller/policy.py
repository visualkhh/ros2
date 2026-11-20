"""학습한 정책(policy.onnx)을 ONNX Runtime으로 돌린다 (실기 rl_nav_controller와 같은 런타임). onnxruntime/numpy는 이 파일에서만 쓴다."""
from pathlib import Path


class OnnxPolicy:
    """obs(리스트) -> 행동(리스트). policy.onnx 옆의 policy.onnx.data(가중치)도 ONNX Runtime이 같이 읽는다."""

    def __init__(self, path: Path) -> None:
        import numpy  # noqa: F401  # 없으면 여기서 ImportError: 정책 없이 fallback으로 돈다
        import onnxruntime

        self._session = onnxruntime.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        first = self._session.get_inputs()[0]
        self._input = first.name
        self.inputs = int(first.shape[-1])  # 관측 차원 (52 = inflation 입력이 있는 정책)

    def run(self, obs: list[float]) -> list[float]:
        import numpy

        out = self._session.run(None, {self._input: numpy.asarray([obs], dtype=numpy.float32)})[0]
        return [float(v) for v in out[0]]
