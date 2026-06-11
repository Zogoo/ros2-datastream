"""Small HTTP API exposed by the AI worker (proxied by nginx under /api/):

  POST   /upload           image bytes -> run detection, returns detections JSON
  GET    /profiles         current per-class HSV detection profiles
  POST   /profiles/<cls>   image bytes -> resample the HSV band for that class
  DELETE /profiles/<cls>   restore the default band
"""
from __future__ import annotations

import json
import threading
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np


def start_http_api(
    port: int,
    detector,
    on_detections: Callable[[list[dict]], None],
) -> HTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def do_OPTIONS(self) -> None:
            self._respond(204, b"")

        def do_GET(self) -> None:
            if self.path.rstrip("/") == "/profiles":
                self._json(200, detector.profiles_snapshot())
            else:
                self._json(404, {"error": "not found"})

        def do_POST(self) -> None:
            img = self._read_image()
            if self.path == "/upload":
                if img is None:
                    self._json(400, {"error": "cannot decode image"})
                    return
                detections = detector.detect(img)
                on_detections(detections)
                self._json(200, {"objects": detections})
                return
            if self.path.startswith("/profiles/"):
                cls = self.path.rsplit("/", 1)[-1]
                if img is None:
                    self._json(400, {"error": "cannot decode image"})
                    return
                self._json(200, detector.resample_profile(cls, img))
                return
            self._json(404, {"error": "not found"})

        def do_DELETE(self) -> None:
            if self.path.startswith("/profiles/"):
                cls = self.path.rsplit("/", 1)[-1]
                ok = detector.reset_profile(cls)
                self._json(200 if ok else 404, {"reset": ok, "class": cls})
                return
            self._json(404, {"error": "not found"})

        def _read_image(self) -> np.ndarray | None:
            length = int(self.headers.get("Content-Length", 0))
            if length == 0:
                return None
            raw = self.rfile.read(length)
            return cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)

        def _json(self, code: int, payload: dict | list) -> None:
            self._respond(code, json.dumps(payload).encode(), "application/json")

        def _respond(self, code: int, body: bytes, ctype: str = "text/plain") -> None:
            self.send_response(code)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            if body:
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            if body:
                self.wfile.write(body)

        def log_message(self, *_args) -> None:
            return

    server = HTTPServer(("", port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server
