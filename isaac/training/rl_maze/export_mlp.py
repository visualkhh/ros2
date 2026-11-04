"""policy.onnx → policy.mlp — nav2 플러그인(ros2_ws/src/rl_nav_controller)이 외부 라이브러리 없이 읽는 MLP 가중치 파일.

  $ISAACLAB_DIR/isaaclab.sh -p training/rl_maze/export_mlp.py artifacts/rl_maze/training/<시각>/exported/policy.onnx
      → 같은 폴더 policy.mlp (onnxruntime 출력과 같은지 확인까지)

ONNX 그래프가 [Gemm|MatMul(+Add)] → Elu → … → [Gemm|MatMul(+Add)] (rsl_rl actor, 정규화 없음)일 때만. 다르면 거부.
파일 (little-endian): b"RLMLP1\\0\\0" · uint32 층 수 · uint32 활성(1 = ELU, 마지막 층 빼고) · 층마다 uint32 out · uint32 in · float32 W[out×in] · float32 b[out]
"""
from __future__ import annotations

import struct
import sys

import numpy as np
import onnx
from onnx import numpy_helper

MAGIC, ELU = b"RLMLP1\0\0", 1


def layers_of(model) -> list[tuple[np.ndarray, np.ndarray]]:
    init = {t.name: numpy_helper.to_array(t) for t in model.graph.initializer}
    layers, pending, acts = [], None, []
    for node in model.graph.node:
        op = node.op_type
        if op == "Gemm":
            a = {x.name: onnx.helper.get_attribute_value(x) for x in node.attribute}
            w, b = init[node.input[1]], init[node.input[2]] if len(node.input) > 2 else None
            w = w if a.get("transB", 0) else w.T            # → [out, in]
            layers.append((w.astype(np.float32), (b if b is not None else np.zeros(w.shape[0])).astype(np.float32)))
        elif op == "MatMul":
            pending = init[node.input[1]].T.astype(np.float32)   # x @ W → W.T = [out, in]
            layers.append((pending, np.zeros(pending.shape[0], dtype=np.float32)))
        elif op == "Add" and pending is not None:
            b = init.get(node.input[1], init.get(node.input[0]))
            layers[-1] = (layers[-1][0], b.astype(np.float32).reshape(-1))
            pending = None
        elif op == "Elu":
            acts.append(len(layers))
        elif op in ("Identity", "Constant", "Cast"):
            continue
        else:
            raise SystemExit(f"모르는 연산 {op} — MLP(Gemm/MatMul+Add · Elu)만")
    if acts != list(range(1, len(layers))):
        raise SystemExit(f"ELU 위치가 층 사이가 아님: {acts} (층 {len(layers)})")
    return layers


def forward(layers, x: np.ndarray) -> np.ndarray:
    for i, (w, b) in enumerate(layers):
        x = x @ w.T + b
        if i < len(layers) - 1:
            x = np.where(x > 0, x, np.expm1(x))
    return x


def main(path: str) -> str:
    model = onnx.load(path)
    layers = layers_of(model)
    out = path[:-5] + ".mlp" if path.endswith(".onnx") else path + ".mlp"
    with open(out, "wb") as f:
        f.write(MAGIC + struct.pack("<II", len(layers), ELU))
        for w, b in layers:
            f.write(struct.pack("<II", *w.shape))
            f.write(np.ascontiguousarray(w, dtype="<f4").tobytes())
            f.write(np.ascontiguousarray(b, dtype="<f4").tobytes())
    n_in = layers[0][0].shape[1]
    try:   # 같은 입력에서 onnxruntime과 비교
        import onnxruntime as ort

        sess = ort.InferenceSession(path)
        xs = np.random.default_rng(0).standard_normal((32, 1, n_in)).astype(np.float32)   # 입력 배치가 1로 고정
        name = sess.get_inputs()[0].name
        err = max(float(np.abs(sess.run(None, {name: x})[0] - forward(layers, x)).max()) for x in xs)
        assert err < 1e-4, err
        check = f"onnxruntime과 최대 차이 {err:.1e}"
    except ImportError:
        check = "onnxruntime 없음 — 비교 생략"
    print(f"[mlp] {out} · 층 {[w.shape for w, _ in layers]} · {check}")
    return out


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("사용: export_mlp.py <policy.onnx>")
    main(sys.argv[1])
