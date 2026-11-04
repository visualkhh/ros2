"""ONNX 도구 — 학습 체크포인트 → policy.onnx (공식 play.py로 내보내기) · INT8 양자화 + fp32와 오차 비교.

  ../run-make-onnx.sh <model_N.pt> [--quantize]      (= isaac venv python tools/onnx_tool.py export …)
  ../run-make-onnx.sh --quantize-only <policy.onnx>

export   task = 경로에서 (artifacts/<task>/training/<시각>_<label>/model_N.pt) → ./run-training.sh <task> --play
         (창 없음 · 1대) → <같은 폴더>/exported/policy.onnx (+ .onnx.data · policy.pt) 가 생기면 play를 끈다.
quantize onnxruntime.quantization 동적 INT8 (가중치만) → policy.int8.onnx + 같은 입력에서 fp32와 최대 차이.
         작은 MLP(수십 KB)는 이득이 거의 없다 — 비교용. 로봇에는 fp32가 기본.
isaac venv(onnx · onnxruntime)에서 돈다 — run-make-onnx.sh가 그 python으로 부른다.
"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time

ISAAC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def task_of(pt: str) -> str:
    """…/artifacts/<task>/training/<run>/model_N.pt → <task>."""
    parts = os.path.abspath(pt).split(os.sep)
    i = len(parts) - 1 - parts[::-1].index("training") if "training" in parts else -1   # 마지막 training
    if i < 2 or parts[i - 2] != "artifacts":
        raise SystemExit(f"task를 경로에서 못 찾음 (artifacts/<task>/training/<시각>/model_N.pt여야): {pt}")
    return parts[i - 1]


def export(pt: str, timeout: float = 600.0) -> str:
    pt = os.path.abspath(pt)
    if not os.path.isfile(pt):
        raise SystemExit(f"체크포인트 없음: {pt}")
    task = task_of(pt)
    out = os.path.join(os.path.dirname(pt), "exported", "policy.onnx")
    t0 = time.time()
    print(f"[onnx] {task} · {os.path.basename(pt)} → play (창 없음, 1대)", flush=True)
    p = subprocess.Popen([os.path.join(ISAAC, "run-training.sh"), task, "--play", "--checkpoint", pt, "--num_envs", "1"],
                         cwd=ISAAC, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        while time.time() - t0 < timeout:
            if os.path.isfile(out) and os.path.getmtime(out) >= t0 and time.time() - os.path.getmtime(out) > 2.0:
                break   # 다 쓴 뒤 (2초 더 기다림 — .onnx.data까지)
            if p.poll() is not None:
                raise SystemExit(f"play가 먼저 끝남 (rc={p.returncode}) — artifacts/{task}/logs/*_play.log 확인")
            time.sleep(1.0)
        else:
            raise SystemExit(f"{timeout:.0f}초 안에 {out}이 안 생김")
    finally:
        for sig in (signal.SIGINT, signal.SIGKILL):   # play는 스스로 안 끝난다
            try:
                os.killpg(p.pid, sig)
                p.wait(timeout=15)
                break
            except (ProcessLookupError, subprocess.TimeoutExpired):
                continue
    print(f"[onnx] {out}" + (" (+ policy.onnx.data)" if os.path.isfile(out + ".data") else ""), flush=True)
    return out


def compare(a: str, b: str, n: int = 64) -> float:
    """같은 무작위 입력에서 두 ONNX 출력의 최대 차이 (입력 배치 1)."""
    import numpy as np
    import onnxruntime as ort

    sa, sb = ort.InferenceSession(a), ort.InferenceSession(b)
    dim = sa.get_inputs()[0].shape[1]
    xs = np.random.default_rng(0).standard_normal((n, 1, dim)).astype(np.float32)
    ia, ib = sa.get_inputs()[0].name, sb.get_inputs()[0].name
    return max(float(np.abs(sa.run(None, {ia: x})[0] - sb.run(None, {ib: x})[0]).max()) for x in xs)


def quantize(onnx_path: str) -> str:
    from onnxruntime.quantization import QuantType, quantize_dynamic

    onnx_path = os.path.abspath(onnx_path)
    if not os.path.isfile(onnx_path):
        raise SystemExit(f"ONNX 없음: {onnx_path}")
    out = onnx_path[:-5] + ".int8.onnx" if onnx_path.endswith(".onnx") else onnx_path + ".int8.onnx"
    import tempfile

    import onnx

    m = onnx.load(onnx_path)            # .onnx.data(가중치)까지 읽힌다
    del m.graph.value_info[:]           # Isaac Lab이 남긴 중간 모양 정보가 가중치와 안 맞아 양자화의 모양 검사가 멈춘다 — 지우고
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, "fp32.onnx")
        onnx.save(m, src)               # 한 파일로
        quantize_dynamic(src, out, weight_type=QuantType.QInt8)
    err = compare(onnx_path, out)
    kb = lambda f: sum(os.path.getsize(x) for x in (f, f + ".data") if os.path.isfile(x)) / 1024  # noqa: E731
    print(f"[onnx] {out} · {kb(onnx_path):.0f} KB → {kb(out):.0f} KB · fp32와 최대 차이 {err:.2e} (정책 출력 [-1, 1] 기준)")
    return out


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="run-make-onnx.sh", description="체크포인트 → policy.onnx (+ INT8 양자화)")
    p.add_argument("path", help="model_N.pt (export) · --quantize-only면 policy.onnx")
    p.add_argument("--quantize", action="store_true", help="내보낸 뒤 INT8 양자화 (policy.int8.onnx) + 오차 비교")
    p.add_argument("--quantize-only", action="store_true", help="이미 있는 policy.onnx만 양자화")
    args = p.parse_args(argv)
    if args.quantize_only:
        quantize(args.path)
        return 0
    out = export(args.path)
    if args.quantize:
        quantize(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
