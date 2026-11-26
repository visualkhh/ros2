#!/usr/bin/env python3
"""Serve SmolVLM2-500M image Q&A over local HTTP for the krosc vlm_node.

  krosc_ws/tools/vla-venv/bin/python krosc_ws/tools/vlm_server.py

Endpoints:
  GET  /health
  POST /query  {image_b64, encoding, width, height, question}
"""
import argparse
import base64
import io
import json
import os
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

BASE = Path(__file__).resolve().parents[1]  # krosc_ws
os.environ.setdefault("HF_HOME", str(BASE / "artifacts/models/vlm/hf-cache"))

import numpy as np  # noqa: E402
import torch  # noqa: E402
from PIL import Image  # noqa: E402
from transformers import AutoModelForImageTextToText, AutoProcessor  # noqa: E402

MODEL = "HuggingFaceTB/SmolVLM2-500M-Video-Instruct"
REV = "4cde9d208e19519c36917aa170fffc7e381aa0ad"


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


class Handler(BaseHTTPRequestHandler):
    model = None
    processor = None
    device = "cpu"

    def _reply(self, status, body):
        data = json.dumps(body, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/health":
            self._reply(200, {"ok": True, "model": MODEL})
        else:
            self._reply(404, {"ok": False, "error": "not found"})

    def do_POST(self):
        if self.path != "/query":
            self._reply(404, {"ok": False, "error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            image = image_from_payload(payload)
            question = str(payload.get("question", "")).strip() or "What do you see?"
            conversation = [{
                "role": "user",
                "content": [{"type": "image"}, {"type": "text", "text": question}],
            }]
            prompt = self.processor.apply_chat_template(
                conversation, add_generation_prompt=True, tokenize=False)
            inputs = self.processor(text=prompt, images=[image], return_tensors="pt")
            inputs = {key: value.to(self.device) for key, value in inputs.items()}
            with torch.inference_mode():
                generated = self.model.generate(**inputs, max_new_tokens=96, do_sample=False)
            tokens = generated[:, inputs["input_ids"].shape[1]:]
            answer = self.processor.batch_decode(tokens, skip_special_tokens=True)[0].strip()
            self._reply(200, {"ok": True, "answer": answer})
        except Exception as error:  # noqa: BLE001 - report model/input errors to the node
            self._reply(400, {"ok": False, "error": str(error)})

    def log_message(self, fmt, *args):
        sys.stderr.write("[vlm_server] " + fmt % args + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--device", default="mps" if torch.backends.mps.is_available() else "cpu")
    parser.add_argument("--threads", type=int, default=0)
    args = parser.parse_args()

    if args.threads > 0:
        torch.set_num_threads(args.threads)
    dtype = torch.float16 if args.device == "cuda" else torch.float32
    print(f"[vlm_server] loading {MODEL} on {args.device}", flush=True)
    processor = AutoProcessor.from_pretrained(MODEL, revision=REV)
    processor.image_processor.do_image_splitting = False
    model = AutoModelForImageTextToText.from_pretrained(
        MODEL, revision=REV, dtype=dtype).to(args.device).eval()
    Handler.processor = processor
    Handler.model = model
    Handler.device = args.device
    print(f"[vlm_server] ready http://{args.host}:{args.port}/query", flush=True)
    ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
