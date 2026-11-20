"""계산 백엔드 선택: GPU(cupy) -> CPU(numpy) -> 순수 파이썬 순으로 내려간다.

라즈베리파이처럼 GPU/CUDA/numpy가 없는 곳에서도 그대로 돈다(느릴 뿐). 환경변수 KROS_COMPUTE=auto|gpu|cpu|python 으로 고정할 수 있다.
numpy/cupy는 여기서만, 쓸 때 import한다 (kros 코어는 의존성이 없다).
"""
import logging
import os
from types import ModuleType
from typing import Any

logger = logging.getLogger(__name__)

GPU_MIN_WORK = 50_000  # 레이 수 x 도형 수가 이 이상일 때만 GPU (측정: 이보다 작으면 전송/호출 비용 때문에 CPU가 더 빠르다)
_UNSET: Any = object()


def _import(name: str) -> ModuleType | None:
    try:
        module = __import__(name)
        if name == "cupy":  # import가 돼도 GPU/드라이버가 없으면 쓸 수 없다
            module.zeros(1).sum().item()
        return module
    except Exception:  # ImportError, CUDA 라이브러리/드라이버 없음 등 어떤 이유든
        return None


class Compute:
    """어떤 배열 모듈(xp)로 계산할지 고른다. pick()이 None이면 순수 파이썬으로 계산해야 한다."""

    def __init__(self, mode: str | None = None, numpy: Any = _UNSET, cupy: Any = _UNSET) -> None:
        self.mode = mode or os.environ.get("KROS_COMPUTE", "auto")
        if self.mode not in ("auto", "gpu", "cpu", "python"):
            raise ValueError(f"KROS_COMPUTE는 auto|gpu|cpu|python 중 하나: {self.mode}")
        # 인자로 모듈(또는 None=없음)을 주면 그걸 쓴다 (테스트에서 없는 척하거나 가짜를 끼울 때)
        self.numpy = _import("numpy") if numpy is _UNSET and self.mode != "python" else (None if numpy is _UNSET else numpy)
        self.cupy = _import("cupy") if cupy is _UNSET and self.mode in ("auto", "gpu") and self.numpy is not None else (None if cupy is _UNSET else cupy)

    @property
    def name(self) -> str:
        if self.numpy is None or self.mode == "python":
            return "python"
        return "gpu(cupy)" if self.cupy is not None and self.mode in ("auto", "gpu") else "cpu(numpy)"

    def pick(self, work: int) -> Any:
        """work(레이 수 x 도형 수)에 맞는 배열 모듈. 순수 파이썬으로 가야 하면 None."""
        if self.numpy is None or self.mode == "python":
            return None
        if self.cupy is not None and (self.mode == "gpu" or (self.mode == "auto" and work >= GPU_MIN_WORK)):
            return self.cupy
        return self.numpy

    def disable_gpu(self, reason: Exception) -> None:
        logger.warning("GPU 계산 실패, 이후 CPU(numpy)로 계산한다: %r", reason)
        self.cupy = None

    def to_list(self, array: Any) -> list[float]:
        values: list[float] = array.get().tolist() if self.cupy is not None and isinstance(array, self.cupy.ndarray) else array.tolist()
        return values


compute = Compute()  # 프로세스 공용 (World가 기본으로 쓴다)
