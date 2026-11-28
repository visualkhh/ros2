#!/usr/bin/env python3
"""Serve the waffle-color VLA (SmolVLM2-500M + color head) over local HTTP.

The C++ vla_node posts one observation and receives the 92x7 joint/base plan.
Run on the Mac (CPU/MPS) or the GPU PC with the learning repo on PYTHONPATH:

  srf-default-ws/tools/vla-venv/bin/python srf-default-ws/tools/vla_server.py \
      --learning-dir /Users/hyunhakim/workspace/pa/source/waffle-mpx-lyrical/learning

Endpoints:
  GET  /health
  POST /plan   {image_b64, encoding, width, height, instruction, state[7]}
"""
import argparse
import base64
import io
import json
import os
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

BASE = Path(__file__).resolve().parents[1]  # srf_default_ws
os.environ.setdefault("HF_HOME", str(BASE / "artifacts/models/vla/hf-cache"))

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402


def image_from_payload(payload):
    raw = base64.b64decode(payload["image_b64"])
    encoding = str(payload.get("encoding", "jpeg")).lower()
    if encoding in ("jpeg", "jpg", "png"):
        return Image.open(io.BytesIO(raw)).convert("RGB")
    width = int(payload["width"])
    height = int(payload["height"])
    array = np.frombuffer(raw, dtype=np.uint8)
    if encoding == "bgr8":
        array = array.reshape(height, width, 3)[:, :, ::-1]
    else:
        array = array.reshape(height, width, 3)
    return Image.fromarray(array, "RGB")


def canonical_instruction(text):
    lowered = text.lower()
    if "red" in lowered or "빨" in text:
        return "Pick up the red cube and place it on the green square."
    if "blue" in lowered or "파" in text:
        return "Pick up the blue cube and place it on the green square."
    raise ValueError("red/blue object is required for the color VLA")


class Handler(BaseHTTPRequestHandler):
    policy = None

    def _reply(self, status, body):
        data = json.dumps(body, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/health":
            self._reply(200, {"ok": True, "model": "waffle-color-73"})
        else:
            self._reply(404, {"ok": False, "error": "not found"})

    def do_POST(self):
        if self.path != "/plan":
            self._reply(404, {"ok": False, "error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            image = image_from_payload(payload)
            instruction = canonical_instruction(payload.get("instruction", ""))
            state = [float(value) for value in payload["state"]]
            if len(state) != 7:
                raise ValueError(f"state must have 7 values, got {len(state)}")
            actions = np.asarray(self.policy.plan(image, instruction, state), dtype=float)
            if actions.ndim != 2 or actions.shape[1] != 7:
                raise ValueError(f"unexpected action shape {actions.shape}")
            self._reply(200, {
                "ok": True,
                "instruction": instruction,
                "steps": int(actions.shape[0]),
                "actions": actions.tolist(),
                "actions_flat": actions.reshape(-1).tolist(),
            })
        except Exception as error:  # noqa: BLE001 - report model/input errors to the node
            self._reply(400, {"ok": False, "error": str(error)})

    def log_message(self, fmt, *args):
        sys.stderr.write("[vla_server] " + fmt % args + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--learning-dir", type=Path, default=BASE / "tools/learning")
    parser.add_argument("--parent", type=Path,
                        default=BASE / "artifacts/models/vla/waffle-hf-vla.pt")
    parser.add_argument("--color", type=Path,
                        default=BASE / "artifacts/models/vla/waffle-color-73.npz")
    parser.add_argument("--threads", type=int, default=0)
    args = parser.parse_args()

    learning_dir = args.learning_dir.resolve()
    parent = (args.parent or learning_dir / "checkpoints" / "waffle-hf-vla.pt").resolve()
    color = (args.color or learning_dir / "checkpoints" / "waffle-color-73.npz").resolve()
    for path in (learning_dir, parent, color):
        if not path.exists():
            parser.error(f"missing path: {path}")

    if args.threads > 0:
        import torch
        torch.set_num_threads(args.threads)

    sys.path.insert(0, str(learning_dir))
    from color_imitation import ColorPolicy  # noqa: PLC0415 - needs learning-dir on sys.path

    print(f"[vla_server] loading parent={parent.name} color={color.name}", flush=True)
    Handler.policy = ColorPolicy(color, parent)
    print(f"[vla_server] ready http://{args.host}:{args.port}/plan", flush=True)
    ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
