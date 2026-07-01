#!/usr/bin/env python3
"""View the Booster OS app camera stream from a laptop browser.

The Booster app camera feed is exposed by the robot as binary WebSocket frames
on TCP port 51111. Each frame contains a protobuf-style envelope with a JPEG
payload inside it. This helper keeps the robot connection local to the laptop
and republishes the latest JPEGs as a simple MJPEG browser stream.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import http.server
import json
import os
import subprocess
import socket
import struct
import sys
import threading
import time
import webbrowser
from dataclasses import dataclass, field
from typing import Optional, Union


JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"
DETECTION_FIELDS = {"label", "confidence", "xmin", "ymin", "xmax", "ymax"}


@dataclass
class Detection:
    label: str
    confidence: float
    xmin: int
    ymin: int
    xmax: int
    ymax: int

    def as_dict(self) -> dict[str, object]:
        return {
            "label": self.label,
            "confidence": self.confidence,
            "xmin": self.xmin,
            "ymin": self.ymin,
            "xmax": self.xmax,
            "ymax": self.ymax,
        }

    def is_ball(self) -> bool:
        return self.label.strip().lower() == "ball"


@dataclass
class FrameStore:
    condition: threading.Condition = field(default_factory=threading.Condition)
    jpeg: Optional[bytes] = None
    detections: list[Detection] = field(default_factory=list)
    yolo_only_ball: bool = False
    frame_id: int = 0
    last_error: str = ""
    last_update: float = 0.0
    connected: bool = False
    detections_connected: bool = False
    detections_last_error: str = ""
    detections_last_update: float = 0.0

    def set_frame(self, jpeg: bytes) -> None:
        with self.condition:
            self.jpeg = jpeg
            self.frame_id += 1
            self.last_update = time.time()
            self.last_error = ""
            self.connected = True
            self.condition.notify_all()

    def set_status(self, *, connected: Optional[bool] = None, error: str = "") -> None:
        with self.condition:
            if connected is not None:
                self.connected = connected
            if error:
                self.last_error = error
            self.condition.notify_all()

    def set_detections(self, detections: list[Detection]) -> None:
        with self.condition:
            self.detections = (
                [detection for detection in detections if detection.is_ball()]
                if self.yolo_only_ball
                else detections
            )
            self.detections_last_update = time.time()
            self.detections_last_error = ""
            self.detections_connected = True
            self.condition.notify_all()

    def set_detections_status(self, *, connected: Optional[bool] = None, error: str = "") -> None:
        with self.condition:
            if connected is not None:
                self.detections_connected = connected
            if error:
                self.detections_last_error = error
            self.condition.notify_all()

    def snapshot(self) -> tuple[int, Optional[bytes], dict[str, object]]:
        with self.condition:
            age = time.time() - self.last_update if self.last_update else None
            detections_age = (
                time.time() - self.detections_last_update if self.detections_last_update else None
            )
            stats = {
                "connected": self.connected,
                "frame_id": self.frame_id,
                "last_frame_age_sec": age,
                "last_error": self.last_error,
                "detections_connected": self.detections_connected,
                "detections_count": len(self.detections),
                "detections_last_update_age_sec": detections_age,
                "detections_last_error": self.detections_last_error,
            }
            return self.frame_id, self.jpeg, stats

    def detections_snapshot(self) -> dict[str, object]:
        with self.condition:
            age = (
                time.time() - self.detections_last_update if self.detections_last_update else None
            )
            return {
                "detections": [detection.as_dict() for detection in self.detections],
                "stats": {
                    "connected": self.detections_connected,
                    "count": len(self.detections),
                    "last_update_age_sec": age,
                    "last_error": self.detections_last_error,
                },
            }


def recv_exact(sock: socket.socket, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise EOFError("connection closed")
        data.extend(chunk)
    return bytes(data)


def read_ws_frame(sock: socket.socket) -> tuple[int, bytes]:
    header = recv_exact(sock, 2)
    first, second = header
    opcode = first & 0x0F
    masked = bool(second & 0x80)
    length = second & 0x7F

    if length == 126:
        length = struct.unpack("!H", recv_exact(sock, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", recv_exact(sock, 8))[0]

    mask = recv_exact(sock, 4) if masked else b""
    payload = recv_exact(sock, length) if length else b""
    if masked:
        payload = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
    return opcode, payload


def send_ws_close(sock: socket.socket) -> None:
    try:
        sock.sendall(b"\x88\x00")
    except OSError:
        pass


def websocket_connect(host: str, port: int, path: str, timeout: float) -> socket.socket:
    key = base64.b64encode(os.urandom(16)).decode("ascii")
    request = (
        f"GET {path} HTTP/1.1\r\n"
        f"Host: {host}:{port}\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\n"
        "Sec-WebSocket-Version: 13\r\n"
        "\r\n"
    ).encode("ascii")

    sock = socket.create_connection((host, port), timeout=timeout)
    sock.settimeout(timeout)
    sock.sendall(request)

    response = bytearray()
    while b"\r\n\r\n" not in response:
        response.extend(sock.recv(1024))
        if len(response) > 8192:
            raise RuntimeError("oversized websocket handshake response")

    header = bytes(response).split(b"\r\n\r\n", 1)[0]
    first_line = header.split(b"\r\n", 1)[0]
    if b"101" not in first_line:
        raise RuntimeError(first_line.decode("ascii", errors="replace"))

    expected_accept = base64.b64encode(
        hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")).digest()
    )
    if expected_accept not in header:
        raise RuntimeError("websocket accept header did not match")

    return sock


def extract_jpeg(payload: bytes) -> Optional[bytes]:
    start = payload.find(JPEG_SOI)
    if start < 0:
        return None
    end = payload.rfind(JPEG_EOI)
    if end < start:
        return None
    return payload[start : end + len(JPEG_EOI)]


def camera_reader(store: FrameStore, host: str, port: int, path: str, timeout: float) -> None:
    retry_delay = 0.5
    while True:
        sock: Optional[socket.socket] = None
        try:
            sock = websocket_connect(host, port, path, timeout)
            store.set_status(connected=True)
            retry_delay = 0.5
            while True:
                opcode, payload = read_ws_frame(sock)
                if opcode == 0x8:
                    raise EOFError("websocket close frame received")
                if opcode not in (0x1, 0x2):
                    continue
                jpeg = extract_jpeg(payload)
                if jpeg:
                    store.set_frame(jpeg)
        except Exception as exc:  # Keep the browser endpoint alive while reconnecting.
            store.set_status(connected=False, error=str(exc))
            if sock is not None:
                send_ws_close(sock)
                sock.close()
            time.sleep(retry_delay)
            retry_delay = min(retry_delay * 2, 5.0)


class RosDetectionParser:
    def __init__(self, on_message) -> None:
        self.on_message = on_message
        self.objects: list[dict[str, object]] = []
        self.current: Optional[dict[str, object]] = None

    def feed_line(self, line: str) -> None:
        stripped = line.strip()
        if stripped == "---":
            self.commit_message()
            return

        if stripped.startswith("- "):
            rest = stripped[2:].strip()
            if rest.startswith("label:"):
                self.finish_current()
                self.current = {}
                self.parse_field(rest)
            return

        if self.current is not None:
            self.parse_field(stripped)

    def parse_field(self, text: str) -> None:
        if ":" not in text or self.current is None:
            return
        key, raw_value = text.split(":", 1)
        key = key.strip()
        if key not in DETECTION_FIELDS:
            return
        raw_value = raw_value.strip().strip("'\"")
        try:
            if key in {"xmin", "ymin", "xmax", "ymax"}:
                value: object = int(float(raw_value))
            elif key == "confidence":
                value = float(raw_value)
            else:
                value = raw_value
        except ValueError:
            return
        self.current[key] = value

    def finish_current(self) -> None:
        if self.current is not None:
            self.objects.append(self.current)
            self.current = None

    def commit_message(self) -> None:
        self.finish_current()
        detections = []
        for obj in self.objects:
            if not DETECTION_FIELDS.issubset(obj):
                continue
            xmin = int(obj["xmin"])
            ymin = int(obj["ymin"])
            xmax = int(obj["xmax"])
            ymax = int(obj["ymax"])
            if xmax <= xmin or ymax <= ymin:
                continue
            detections.append(
                Detection(
                    label=str(obj["label"]),
                    confidence=float(obj["confidence"]),
                    xmin=xmin,
                    ymin=ymin,
                    xmax=xmax,
                    ymax=ymax,
                )
            )
        self.objects = []
        self.on_message(detections)


def remote_detection_command(topic: str) -> str:
    return (
        "cd ~/booster_soccer || exit 1; "
        "deactivate 2>/dev/null || true; "
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        f"ros2 topic echo {topic}"
    )


def detection_reader(
    store: FrameStore,
    command: Union[str, list[str]],
    *,
    shell: bool,
    restart_delay: float = 2.0,
) -> None:
    while True:
        parser = RosDetectionParser(store.set_detections)
        process: Optional[subprocess.Popen] = None
        try:
            process = subprocess.Popen(
                command,
                shell=shell,
                stdout=subprocess.PIPE,
                stderr=None,
                text=True,
                bufsize=1,
            )
            store.set_detections_status(connected=True)
            assert process.stdout is not None
            for line in process.stdout:
                parser.feed_line(line)
            rc = process.wait()
            store.set_detections_status(
                connected=False,
                error=f"detection command exited with status {rc}",
            )
        except Exception as exc:
            store.set_detections_status(connected=False, error=str(exc))
            if process is not None and process.poll() is None:
                process.terminate()
        time.sleep(restart_delay)


def make_handler(store: FrameStore):
    class Handler(http.server.BaseHTTPRequestHandler):
        server_version = "BoosterCameraViewer/1.0"

        def log_message(self, fmt: str, *args: object) -> None:
            sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))

        def do_GET(self) -> None:
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(INDEX_HTML.encode("utf-8"))
                return

            if self.path == "/stream.mjpg":
                self.stream_mjpeg()
                return

            if self.path == "/snapshot.jpg":
                _, jpeg, _ = store.snapshot()
                if not jpeg:
                    self.send_error(503, "No camera frame yet")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(jpeg)))
                self.end_headers()
                self.wfile.write(jpeg)
                return

            if self.path == "/stats.json":
                _, _, stats = store.snapshot()
                data = json.dumps(stats).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return

            if self.path == "/detections.json":
                data = json.dumps(store.detections_snapshot()).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return

            self.send_error(404)

        def stream_mjpeg(self) -> None:
            boundary = "booster-frame"
            self.send_response(200)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Pragma", "no-cache")
            self.end_headers()

            last_id = -1
            while True:
                with store.condition:
                    store.condition.wait_for(lambda: store.frame_id != last_id, timeout=5.0)
                    frame_id = store.frame_id
                    jpeg = store.jpeg

                if not jpeg or frame_id == last_id:
                    continue

                last_id = frame_id
                part = (
                    f"--{boundary}\r\n"
                    "Content-Type: image/jpeg\r\n"
                    f"Content-Length: {len(jpeg)}\r\n"
                    "\r\n"
                ).encode("ascii")
                try:
                    self.wfile.write(part)
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    return

    return Handler


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Booster Camera</title>
  <style>
    html, body { margin: 0; height: 100%; background: #111; color: #eee; font-family: system-ui, sans-serif; }
    body { display: grid; grid-template-rows: auto 1fr; }
    header { display: flex; align-items: center; justify-content: space-between; gap: 1rem; padding: .6rem .8rem; background: #202020; }
    h1 { margin: 0; font-size: 1rem; font-weight: 600; }
    #stats { font-variant-numeric: tabular-nums; color: #bbb; font-size: .85rem; }
    main { min-height: 0; display: grid; place-items: center; }
    #viewer { position: relative; width: 100%; height: 100%; overflow: hidden; }
    img, svg { position: absolute; inset: 0; display: block; width: 100%; height: 100%; object-fit: contain; }
    svg { pointer-events: none; }
    .box { fill: none; stroke: #00e676; stroke-width: 3; vector-effect: non-scaling-stroke; }
    .box-text { fill: #fff; font-size: 22px; font-weight: 700; paint-order: stroke; stroke: #000; stroke-width: 4px; stroke-linejoin: round; }
    .center-dot { fill: #ff2d16; stroke: #7a0e06; stroke-width: 2; vector-effect: non-scaling-stroke; }
    .center-text { fill: #ffe94a; font-size: 22px; font-weight: 800; paint-order: stroke; stroke: #000; stroke-width: 4px; stroke-linejoin: round; }
  </style>
</head>
<body>
  <header>
    <h1>Booster Camera</h1>
    <div id="stats">connecting...</div>
  </header>
  <main>
    <div id="viewer">
      <img id="camera" src="/stream.mjpg" alt="Booster camera live stream">
      <svg id="overlay" preserveAspectRatio="xMidYMid meet"></svg>
    </div>
  </main>
  <script>
    const img = document.getElementById('camera');
    const overlay = document.getElementById('overlay');
    const svgNS = 'http://www.w3.org/2000/svg';

    function setOverlayViewBox() {
      if (img.naturalWidth && img.naturalHeight) {
        overlay.setAttribute('viewBox', `0 0 ${img.naturalWidth} ${img.naturalHeight}`);
      }
    }

    function makeSvg(tag, attrs) {
      const el = document.createElementNS(svgNS, tag);
      for (const [key, value] of Object.entries(attrs)) {
        el.setAttribute(key, value);
      }
      return el;
    }

    function renderDetections(detections) {
      setOverlayViewBox();
      overlay.replaceChildren();
      for (const det of detections) {
        const x = Number(det.xmin);
        const y = Number(det.ymin);
        const width = Number(det.xmax) - x;
        const height = Number(det.ymax) - y;
        if (!Number.isFinite(x + y + width + height) || width <= 0 || height <= 0) {
          continue;
        }
        const confidence = Number(det.confidence);
        const label = `${det.label ?? 'object'} ${Number.isFinite(confidence) ? confidence.toFixed(2) : ''}`;
        const textY = y > 26 ? y - 7 : y + 25;
        const centerX = Math.round(x + width / 2);
        const centerY = Math.round(y + height / 2);
        const centerTextY = img.naturalHeight && y + height + 34 > img.naturalHeight ? y - 34 : y + height + 34;
        overlay.appendChild(makeSvg('rect', {class: 'box', x, y, width, height, rx: 2}));
        overlay.appendChild(makeSvg('text', {class: 'box-text', x: x + 4, y: textY}, label));
        overlay.lastChild.textContent = label;
        overlay.appendChild(makeSvg('circle', {class: 'center-dot', cx: centerX, cy: centerY, r: 8}));
        overlay.appendChild(makeSvg('text', {class: 'center-text', x: x + 4, y: centerTextY}, `center=(${centerX},${centerY})`));
        overlay.lastChild.textContent = `center=(${centerX},${centerY})`;
      }
    }

    async function updateStats() {
      try {
        const res = await fetch('/stats.json', {cache: 'no-store'});
        const stats = await res.json();
        const age = stats.last_frame_age_sec == null ? 'no frame' : `${stats.last_frame_age_sec.toFixed(1)}s`;
        const detAge = stats.detections_last_update_age_sec == null ? 'no detections' : `${stats.detections_last_update_age_sec.toFixed(1)}s`;
        document.getElementById('stats').textContent =
          `${stats.connected ? 'camera' : 'camera reconnecting'} | frame ${stats.frame_id} | age ${age} | boxes ${stats.detections_count} | det ${detAge}`;
      } catch (_) {
        document.getElementById('stats').textContent = 'stats unavailable';
      }
    }

    async function updateDetections() {
      try {
        const res = await fetch('/detections.json', {cache: 'no-store'});
        const payload = await res.json();
        renderDetections(payload.detections ?? []);
      } catch (_) {
        renderDetections([]);
      }
    }

    updateStats();
    updateDetections();
    setInterval(updateDetections, 100);
    setInterval(updateStats, 1000);
    setInterval(setOverlayViewBox, 1000);
  </script>
</body>
</html>
"""


def str_to_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "y", "on"}:
        return True
    if text in {"0", "false", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {value!r}")


def apply_legacy_key_value_args(args: argparse.Namespace, extras: list[str]) -> None:
    for extra in extras:
        if "=" not in extra:
            raise SystemExit(f"unknown argument: {extra}")
        key, value = extra.split("=", 1)
        normalized_key = key.strip().lower().replace("-", "_")
        if normalized_key != "yolo_only_ball":
            raise SystemExit(f"unknown argument: {extra}")
        args.yolo_only_ball = str_to_bool(value)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default="192.168.68.103", help="robot IP address")
    parser.add_argument("--ws-port", type=int, default=51111, help="robot WebSocket camera port")
    parser.add_argument("--ws-path", default="/", help="robot WebSocket path")
    parser.add_argument(
        "--detections",
        choices=("none", "ssh", "local", "cmd"),
        default="none",
        help="detection source",
    )
    parser.add_argument("--detection-topic", default="/booster_soccer/detection", help="ROS detection topic")
    parser.add_argument("--ssh-target", default="", help="SSH target for --detections ssh, default booster@<robot>")
    parser.add_argument("--detections-cmd", default="", help="local shell command for --detections cmd")
    parser.add_argument("--listen-host", default="127.0.0.1", help="local HTTP host")
    parser.add_argument("--listen-port", type=int, default=8091, help="local HTTP port")
    parser.add_argument("--timeout", type=float, default=5.0, help="robot socket timeout in seconds")
    parser.add_argument("--open", action="store_true", help="open the browser after starting")
    parser.add_argument(
        "--yolo-only-ball",
        nargs="?",
        const=True,
        default=False,
        type=str_to_bool,
        help="show only Ball detections; default shows all YOLO objects",
    )
    args, extras = parser.parse_known_args()
    apply_legacy_key_value_args(args, extras)
    return args


def main() -> int:
    args = parse_args()
    store = FrameStore(yolo_only_ball=args.yolo_only_ball)
    reader = threading.Thread(
        target=camera_reader,
        args=(store, args.robot, args.ws_port, args.ws_path, args.timeout),
        daemon=True,
    )
    reader.start()

    if args.detections != "none":
        if args.detections == "ssh":
            ssh_target = args.ssh_target or f"booster@{args.robot}"
            detection_command: Union[str, list[str]] = [
                "ssh",
                ssh_target,
                "bash",
                "-lc",
                remote_detection_command(args.detection_topic),
            ]
            detection_shell = False
            print(f"Detection source: ssh {ssh_target} {args.detection_topic}", flush=True)
        elif args.detections == "local":
            detection_command = ["bash", "-lc", remote_detection_command(args.detection_topic)]
            detection_shell = False
            print(f"Detection source: local ROS topic {args.detection_topic}", flush=True)
        else:
            if not args.detections_cmd:
                print("--detections cmd requires --detections-cmd", file=sys.stderr)
                return 2
            detection_command = args.detections_cmd
            detection_shell = True
            print(f"Detection source command: {args.detections_cmd}", flush=True)

        detections = threading.Thread(
            target=detection_reader,
            args=(store, detection_command),
            kwargs={"shell": detection_shell},
            daemon=True,
        )
        detections.start()

    handler = make_handler(store)
    server = http.server.ThreadingHTTPServer((args.listen_host, args.listen_port), handler)
    url = f"http://{args.listen_host}:{args.listen_port}/"
    print(f"Booster camera viewer: {url}", flush=True)
    print(f"Robot WebSocket source: ws://{args.robot}:{args.ws_port}{args.ws_path}", flush=True)
    if args.open:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping viewer.", flush=True)
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
