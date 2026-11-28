#!/usr/bin/env python3
"""LLM gateway for srf: proxy Ollama /api/chat and add RAG context later.

The robot (Pi) talks to this server instead of Ollama directly:

    robot  --HTTP-->  llm_server.py  --HTTP-->  Ollama (:11434)
                         |
                         +-- RAG hook (future)

Endpoints:
    GET  /health
    GET  /api/tags
    POST /api/chat
    POST /rag/query      (future; returns 501 for now)

Run on the PC:
    srf-default-ws/tools/vla-venv/bin/python srf-default-ws/tools/llm_server.py \
        --host 0.0.0.0 --port 8767

Then on the robot:
    SRF_DEFAULT_WS_OLLAMA_URL=http://<PC-IP>:8767
"""
import argparse
import json
import os
import sys
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

DEFAULT_BACKEND = os.environ.get("SRF_DEFAULT_WS_OLLAMA_BACKEND", "http://127.0.0.1:11434")
DEFAULT_RAG_URL = os.environ.get("SRF_DEFAULT_WS_RAG_URL", "")
DEFAULT_MODEL = os.environ.get("SRF_DEFAULT_WS_LLM_MODEL", "gemma3:12b")


def http_json(url, method="GET", payload=None, timeout=120.0):
    data = None
    headers = {"Accept": "application/json"}
    if payload is not None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        headers["Content-Type"] = "application/json; charset=utf-8"
    request = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(request, timeout=timeout) as response:
        body = response.read()
        return response.status, body


def rag_context(payload):
    """Return extra context to inject into the system message.

    This is the single integration point for the future RAG service. For now it
    is intentionally a no-op so routing behavior is unchanged. A real version
    can call DEFAULT_RAG_URL with the latest user message and return retrieved
    documents, or read a local index.
    """
    del payload
    return ""


def augment_chat(payload):
    context = rag_context(payload)
    if not context:
        return payload
    messages = payload.get("messages")
    if not isinstance(messages, list):
        messages = []
    messages.insert(0, {"role": "system", "content": context})
    payload["messages"] = messages
    return payload


class Handler(BaseHTTPRequestHandler):
    backend = DEFAULT_BACKEND
    model = DEFAULT_MODEL
    timeout_s = 120.0

    def _reply(self, status, body, content_type="application/json; charset=utf-8"):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _error(self, status, message):
        self._reply(status, json.dumps({"ok": False, "error": message},
                                       ensure_ascii=False))

    def _proxy(self, path, method="GET", payload=None):
        url = self.backend.rstrip("/") + path
        try:
            status, body = http_json(url, method=method, payload=payload,
                                     timeout=self.timeout_s)
            self._reply(status, body)
        except urllib.error.HTTPError as error:
            body = error.read()
            self._reply(error.code, body or json.dumps(
                {"ok": False, "error": str(error)}).encode("utf-8"))
        except Exception as error:  # noqa: BLE001 - surface backend failure to the robot
            self._error(502, f"Ollama backend error: {error}")

    def do_GET(self):
        if self.path == "/health":
            backend_ok = False
            backend_error = ""
            try:
                status, _ = http_json(self.backend.rstrip("/") + "/api/tags",
                                      timeout=2.0)
                backend_ok = 200 <= status < 300
            except Exception as error:  # noqa: BLE001
                backend_error = str(error)
            self._reply(200, json.dumps({
                "ok": True,
                "service": "srf-llm-gateway",
                "model": self.model,
                "backend": self.backend,
                "backend_ok": backend_ok,
                "backend_error": backend_error,
                "rag": bool(DEFAULT_RAG_URL),
            }, ensure_ascii=False))
            return
        if self.path == "/api/tags":
            # 모델 선택은 gateway가 소유한다. Pi에는 gateway 모델만 노출한다.
            self._reply(200, json.dumps({
                "models": [{
                    "name": self.model,
                    "model": self.model,
                    "modified_at": "",
                    "size": 0,
                    "digest": "",
                    "details": {"format": "gguf", "family": "gateway"},
                }],
            }, ensure_ascii=False))
            return
        if self.path == "/":
            self._reply(200, json.dumps({
                "ok": True,
                "service": "srf-llm-gateway",
                "endpoints": ["/health", "/api/tags", "/api/chat"],
            }, ensure_ascii=False))
            return
        self._error(404, "not found")

    def do_POST(self):
        if self.path == "/rag/query":
            self._error(501, "RAG is not configured yet")
            return
        if self.path != "/api/chat":
            self._error(404, "not found")
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("body must be a JSON object")
            # Pi가 보낸 model은 무시하고 gateway 설정을 사용한다.
            payload["model"] = self.model
            payload = augment_chat(payload)
            self._proxy("/api/chat", method="POST", payload=payload)
        except Exception as error:  # noqa: BLE001 - report request errors to the robot
            self._error(400, f"bad request: {error}")

    def log_message(self, fmt, *args):
        sys.stderr.write("[llm_server] " + fmt % args + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--backend", default=DEFAULT_BACKEND,
                        help="Ollama base URL (default: %(default)s)")
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--model", default=DEFAULT_MODEL,
                        help="Ollama model owned by this gateway (default: %(default)s)")
    parser.add_argument("--threads", type=int, default=0,
                        help="ignored (compatibility with vlm/vla sidecar launcher)")
    args = parser.parse_args()

    Handler.backend = args.backend
    Handler.model = args.model
    Handler.timeout_s = max(1.0, args.timeout)
    print(f"[llm_server] backend={Handler.backend} model={Handler.model}", flush=True)
    print(f"[llm_server] ready http://{args.host}:{args.port}/api/chat", flush=True)
    ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
