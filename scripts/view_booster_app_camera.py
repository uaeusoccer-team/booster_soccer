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
import math
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

try:
    import numpy as np
except ImportError:  # The camera stream still works when ROS/numpy is unavailable.
    np = None

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
except ImportError:  # Depth overlay is optional and only starts when requested.
    rclpy = None
    Node = object
    qos_profile_sensor_data = None
    CameraInfo = None
    Image = None

try:
    from vision_interface.msg import Ball as VisionBall
    from vision_interface.msg import Detections as VisionDetections
except ImportError:
    VisionBall = None
    VisionDetections = None

try:
    import yaml
except ImportError:
    yaml = None


JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"
DETECTION_FIELDS = {"label", "confidence", "xmin", "ymin", "xmax", "ymax"}
DETECTION_VECTOR_FIELDS = {"position", "position_projection"}
DEFAULT_DEPTH_TOPIC = "/boostercamera/head/depth"
DEFAULT_DEPTH_CAMERA_INFO_TOPIC = "/boostercamera/head/depth/camera_info"


@dataclass
class Detection:
    label: str
    confidence: float
    xmin: int
    ymin: int
    xmax: int
    ymax: int
    position: list[float] = field(default_factory=list)
    position_projection: list[float] = field(default_factory=list)

    def as_dict(self) -> dict[str, object]:
        return {
            "label": self.label,
            "confidence": self.confidence,
            "xmin": self.xmin,
            "ymin": self.ymin,
            "xmax": self.xmax,
            "ymax": self.ymax,
            "position": self.position,
            "position_projection": self.position_projection,
        }

    def is_ball(self) -> bool:
        return self.label.strip().lower() == "ball"


@dataclass
class BallObservation:
    """Recent Ball position from the vision pipeline, used to gate depth obstacles."""

    x: float
    y: float
    confidence: float
    updated_at: float


@dataclass
class CameraIntrinsics:
    """Pinhole camera intrinsics used to turn depth pixels into 3D rays."""

    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_camera_info(cls, msg: CameraInfo) -> "CameraIntrinsics":
        return cls(fx=msg.k[0], fy=msg.k[4], cx=msg.k[2], cy=msg.k[5])


@dataclass
class DepthPixelMap:
    """Projected depth samples plus the source image pixels they came from."""

    local_points: object
    pixel_u: object
    pixel_v: object
    image_width: int
    image_height: int


@dataclass
class DepthRegion:
    """One drawable depth classification region for the browser overlay."""

    kind: str
    polygon: list[list[int]]
    count: int
    mean_x: float
    mean_y: float
    nearest_x: float
    max_height: float
    source_width: int
    source_height: int
    front: bool = False
    mesh_lines: list[list[list[int]]] = field(default_factory=list)

    def as_dict(self) -> dict[str, object]:
        return {
            "kind": self.kind,
            "polygon": self.polygon,
            "count": self.count,
            "mean_x": self.mean_x,
            "mean_y": self.mean_y,
            "nearest_x": self.nearest_x,
            "max_height": self.max_height,
            "source_width": self.source_width,
            "source_height": self.source_height,
            "front": self.front,
            "mesh_lines": self.mesh_lines,
        }


@dataclass
class FrameStore:
    condition: threading.Condition = field(default_factory=threading.Condition)
    jpeg: Optional[bytes] = None
    detections: list[Detection] = field(default_factory=list)
    depth_regions: list[DepthRegion] = field(default_factory=list)
    yolo_only_ball: bool = False
    frame_id: int = 0
    last_error: str = ""
    last_update: float = 0.0
    connected: bool = False
    detections_connected: bool = False
    detections_last_error: str = ""
    detections_last_update: float = 0.0
    depth_connected: bool = False
    depth_last_error: str = ""
    depth_last_update: float = 0.0
    frame_width: int = 0
    frame_height: int = 0

    def set_frame(self, jpeg: bytes) -> None:
        with self.condition:
            self.jpeg = jpeg
            width, height = jpeg_dimensions(jpeg)
            if width and height:
                self.frame_width = width
                self.frame_height = height
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

    def set_depth_regions(self, regions: list[DepthRegion]) -> None:
        with self.condition:
            self.depth_regions = self.filter_ball_obstacle_regions(regions)
            self.depth_last_update = time.time()
            self.depth_last_error = ""
            self.depth_connected = True
            self.condition.notify_all()

    def filter_ball_obstacle_regions(self, regions: list[DepthRegion]) -> list[DepthRegion]:
        ball_detections = [detection for detection in self.detections if detection.is_ball()]
        if not ball_detections or not self.frame_width or not self.frame_height:
            return regions

        filtered = []
        for region in regions:
            if region.kind == "obstacle" and any(
                self.region_matches_ball(region, ball) for ball in ball_detections
            ):
                continue
            filtered.append(region)
        return filtered

    def region_matches_ball(self, region: DepthRegion, ball: Detection) -> bool:
        region_box = self.scaled_region_box(region)
        ball_box = (ball.xmin, ball.ymin, ball.xmax, ball.ymax)
        overlap = self.box_intersection_area(region_box, ball_box)
        if overlap <= 0:
            return False

        region_area = self.box_area(region_box)
        ball_area = self.box_area(ball_box)
        center_x = (region_box[0] + region_box[2]) / 2.0
        center_y = (region_box[1] + region_box[3]) / 2.0
        padded_ball = self.expand_box(ball_box, 0.20)
        center_inside_ball = (
            padded_ball[0] <= center_x <= padded_ball[2]
            and padded_ball[1] <= center_y <= padded_ball[3]
        )

        overlap_ratio = overlap / min(region_area, ball_area)
        ball_sized_depth_region = region_area <= ball_area * 8.0
        return ball_sized_depth_region and (overlap_ratio >= 0.12 or center_inside_ball)

    def scaled_region_box(self, region: DepthRegion) -> tuple[float, float, float, float]:
        source_width = region.source_width or self.frame_width
        source_height = region.source_height or self.frame_height
        scale_x = self.frame_width / source_width if source_width else 1.0
        scale_y = self.frame_height / source_height if source_height else 1.0
        xs = [point[0] * scale_x for point in region.polygon]
        ys = [point[1] * scale_y for point in region.polygon]
        return min(xs), min(ys), max(xs), max(ys)

    def expand_box(
        self, box: tuple[float, float, float, float], fraction: float
    ) -> tuple[float, float, float, float]:
        width = box[2] - box[0]
        height = box[3] - box[1]
        pad_x = width * fraction
        pad_y = height * fraction
        return box[0] - pad_x, box[1] - pad_y, box[2] + pad_x, box[3] + pad_y

    def box_intersection_area(
        self,
        first: tuple[float, float, float, float],
        second: tuple[float, float, float, float],
    ) -> float:
        x0 = max(first[0], second[0])
        y0 = max(first[1], second[1])
        x1 = min(first[2], second[2])
        y1 = min(first[3], second[3])
        if x1 <= x0 or y1 <= y0:
            return 0.0
        return (x1 - x0) * (y1 - y0)

    def box_area(self, box: tuple[float, float, float, float]) -> float:
        return max(1.0, box[2] - box[0]) * max(1.0, box[3] - box[1])

    def set_depth_status(self, *, connected: Optional[bool] = None, error: str = "") -> None:
        with self.condition:
            if connected is not None:
                self.depth_connected = connected
            if error:
                self.depth_last_error = error
            self.condition.notify_all()

    def snapshot(self) -> tuple[int, Optional[bytes], dict[str, object]]:
        with self.condition:
            age = time.time() - self.last_update if self.last_update else None
            detections_age = (
                time.time() - self.detections_last_update if self.detections_last_update else None
            )
            depth_age = time.time() - self.depth_last_update if self.depth_last_update else None
            stats = {
                "connected": self.connected,
                "frame_id": self.frame_id,
                "last_frame_age_sec": age,
                "last_error": self.last_error,
                "detections_connected": self.detections_connected,
                "detections_count": len(self.detections),
                "detections_last_update_age_sec": detections_age,
                "detections_last_error": self.detections_last_error,
                "depth_connected": self.depth_connected,
                "depth_region_count": len(self.depth_regions),
                "depth_last_update_age_sec": depth_age,
                "depth_last_error": self.depth_last_error,
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

    def depth_snapshot(self) -> dict[str, object]:
        with self.condition:
            age = time.time() - self.depth_last_update if self.depth_last_update else None
            return {
                "regions": [region.as_dict() for region in self.depth_regions],
                "stats": {
                    "connected": self.depth_connected,
                    "count": len(self.depth_regions),
                    "last_update_age_sec": age,
                    "last_error": self.depth_last_error,
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


def jpeg_dimensions(jpeg: bytes) -> tuple[int, int]:
    """Read width/height from a JPEG SOF marker without decoding the image."""

    if not jpeg.startswith(JPEG_SOI):
        return 0, 0

    index = 2
    sof_markers = {
        0xC0,
        0xC1,
        0xC2,
        0xC3,
        0xC5,
        0xC6,
        0xC7,
        0xC9,
        0xCA,
        0xCB,
        0xCD,
        0xCE,
        0xCF,
    }
    standalone_markers = {0x01, *range(0xD0, 0xD8), 0xD8, 0xD9}

    while index + 3 < len(jpeg):
        if jpeg[index] != 0xFF:
            index += 1
            continue
        while index < len(jpeg) and jpeg[index] == 0xFF:
            index += 1
        if index >= len(jpeg):
            break

        marker = jpeg[index]
        index += 1
        if marker in standalone_markers:
            continue
        if index + 2 > len(jpeg):
            break

        segment_length = int.from_bytes(jpeg[index : index + 2], "big")
        if segment_length < 2 or index + segment_length > len(jpeg):
            break
        if marker in sof_markers and segment_length >= 7:
            height = int.from_bytes(jpeg[index + 3 : index + 5], "big")
            width = int.from_bytes(jpeg[index + 5 : index + 7], "big")
            return width, height
        index += segment_length

    return 0, 0


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
    """Parse `ros2 topic echo` text into lightweight Detection objects.

    The viewer is mostly standard-library code, so the YOLO layer can work by
    shelling out to `ros2 topic echo` instead of importing the custom detection
    message package into this script.
    """

    def __init__(self, on_message) -> None:
        self.on_message = on_message
        self.objects: list[dict[str, object]] = []
        self.current: Optional[dict[str, object]] = None
        self.active_vector_field: Optional[str] = None

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
                self.active_vector_field = None
                self.parse_field(rest)
            elif self.current is not None and self.active_vector_field in DETECTION_VECTOR_FIELDS:
                self.parse_vector_item(rest)
            return

        if self.current is not None:
            self.parse_field(stripped)

    def parse_field(self, text: str) -> None:
        if ":" not in text or self.current is None:
            return
        key, raw_value = text.split(":", 1)
        key = key.strip()
        if key not in DETECTION_FIELDS and key not in DETECTION_VECTOR_FIELDS:
            self.active_vector_field = None
            return
        raw_value = raw_value.strip().strip("'\"")
        if key in DETECTION_VECTOR_FIELDS:
            self.active_vector_field = key
            self.current.setdefault(key, [])
            values = self.parse_vector_value(raw_value)
            if values:
                self.current[key] = values
                self.active_vector_field = None
            return

        self.active_vector_field = None
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

    def parse_vector_value(self, raw_value: str) -> list[float]:
        text = raw_value.strip()
        if not text or text == "[]":
            return []
        text = text.strip("[]")
        values = []
        for item in text.replace(",", " ").split():
            try:
                values.append(float(item))
            except ValueError:
                return []
        return values

    def parse_vector_item(self, text: str) -> None:
        if self.current is None or self.active_vector_field is None:
            return
        try:
            value = float(text.rstrip(","))
        except ValueError:
            return
        values = self.current.setdefault(self.active_vector_field, [])
        if isinstance(values, list):
            values.append(value)

    def finish_current(self) -> None:
        if self.current is not None:
            self.objects.append(self.current)
            self.current = None
            self.active_vector_field = None

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
                    position=list(obj.get("position", [])),
                    position_projection=list(obj.get("position_projection", [])),
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


def load_vision_config(path: str) -> dict[str, object]:
    """Load camera defaults from vision.yaml when the file is available."""

    if not path or yaml is None or not os.path.exists(path):
        return {}
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
    except Exception:
        return {}
    return data.get("camera", {}) or {}


def matrix_from_config(camera_config: dict[str, object]) -> Optional[object]:
    """Return the 4x4 camera extrinsic matrix from vision.yaml, if present."""

    if np is None:
        return None
    extrin = camera_config.get("extrin")
    if not extrin:
        return None
    matrix = np.array(extrin, dtype=np.float32)
    if matrix.shape != (4, 4):
        return None
    return matrix


def default_optical_to_local_matrix() -> object:
    """Fallback transform: ROS optical frame -> local forward/left/up frame.

    Depth camera optical frame is normally x-right, y-down, z-forward. The map
    layer uses a robot-like frame because it makes obstacle metrics easier to
    read in the browser:
        local x = forward, local y = left, local z = up
    """

    return np.array(
        [
            [0.0, 0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )


def intrinsics_from_config(camera_config: dict[str, object]) -> Optional[CameraIntrinsics]:
    """Read fallback intrinsics from vision.yaml before CameraInfo arrives."""

    intrin = camera_config.get("intrin")
    if not isinstance(intrin, dict):
        return None
    required = ("fx", "fy", "cx", "cy")
    if not all(name in intrin for name in required):
        return None
    return CameraIntrinsics(
        fx=float(intrin["fx"]),
        fy=float(intrin["fy"]),
        cx=float(intrin["cx"]),
        cy=float(intrin["cy"]),
    )


def image_to_depth_meters(msg: Image) -> object:
    """Decode ROS depth image data into a float32 depth array in meters."""

    encoding = msg.encoding.lower()
    if encoding in ("16uc1", "mono16"):
        dtype = ">u2" if msg.is_bigendian else "<u2"
        row_width = msg.step // np.dtype(dtype).itemsize
        raw = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row_width)
        return raw[:, : msg.width].astype(np.float32) / 1000.0

    if encoding == "32fc1":
        dtype = ">f4" if msg.is_bigendian else "<f4"
        row_width = msg.step // np.dtype(dtype).itemsize
        raw = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row_width)
        return raw[:, : msg.width].astype(np.float32)

    raise ValueError(f"unsupported depth encoding: {msg.encoding}")


class DepthOverlayBuilder:
    """Builds green flat-surface and red obstacle polygons from depth frames."""

    def __init__(self, args: argparse.Namespace, transform: object) -> None:
        self.args = args
        self.transform = transform

    def build_regions(
        self,
        depth_m: object,
        intrinsics: CameraIntrinsics,
        ball_observations: Optional[list[BallObservation]] = None,
    ) -> list[DepthRegion]:
        samples = self.project(depth_m, intrinsics)
        points = samples.local_points
        if points.size == 0:
            return []

        ground_plane = self.fit_ground_plane(samples)
        height_above_ground = self.height_above_ground(points, ground_plane)

        # Height thresholds are relative to the fitted visible floor plane.
        # This lets the overlay follow the floor when the head pitches down.
        flat_mask = height_above_ground >= self.args.depth_ground_min_height
        flat_mask &= height_above_ground <= self.args.depth_ground_max_height

        obstacle_mask = height_above_ground >= self.args.depth_obstacle_min_height
        obstacle_mask &= height_above_ground <= self.args.depth_obstacle_max_height

        regions = []
        flat_mesh = self.ground_mesh_for_mask(samples, flat_mask, height_above_ground)
        if flat_mesh is not None:
            regions.append(flat_mesh)

        obstacle_regions = self.obstacle_regions_for_components(
            samples, obstacle_mask, height_above_ground
        )
        obstacle_regions = [
            region
            for region in obstacle_regions
            if not self.region_matches_ball_observation(region, ball_observations or [])
        ]
        regions.extend(self.merge_obstacle_regions(obstacle_regions))
        return regions

    def region_matches_ball_observation(
        self,
        region: DepthRegion,
        ball_observations: list[BallObservation],
    ) -> bool:
        if region.kind != "obstacle":
            return False

        for ball in ball_observations:
            close_x = abs(region.mean_x - ball.x) <= self.args.depth_ball_filter_x_radius
            close_x |= abs(region.nearest_x - ball.x) <= self.args.depth_ball_filter_x_radius
            close_y = abs(region.mean_y - ball.y) <= self.args.depth_ball_filter_y_radius
            ball_sized = region.count <= self.args.depth_ball_filter_max_region_points
            if close_x and close_y and ball_sized:
                return True
        return False

    def fit_ground_plane(self, samples: DepthPixelMap) -> object:
        """Fit z = ax + by + c for the currently visible floor.

        The T1 head pitch changes the apparent floor slope in camera/head
        coordinates. A fitted plane is more stable than one global floor height.
        """

        if not self.args.depth_auto_ground:
            return np.array([0.0, 0.0, 0.0], dtype=np.float32)

        points = samples.local_points
        if points.shape[0] < 3:
            return np.array([0.0, 0.0, 0.0], dtype=np.float32)

        candidate_indices = self.ground_candidate_indices(samples)
        if candidate_indices.size < 3:
            ground_z = self.estimate_ground_height(samples)
            return np.array([0.0, 0.0, ground_z], dtype=np.float32)

        candidates = points[candidate_indices]
        coeff = self.solve_ground_plane(candidates)
        for _ in range(max(0, self.args.depth_ground_plane_refine_iterations)):
            residual = self.height_above_ground(candidates, coeff)
            inliers = np.abs(residual) <= self.args.depth_ground_plane_inlier_height
            if int(np.count_nonzero(inliers)) < 3:
                break
            coeff = self.solve_ground_plane(candidates[inliers])
        return coeff

    def ground_candidate_indices(self, samples: DepthPixelMap) -> object:
        points = samples.local_points
        v = samples.pixel_v
        min_v = samples.image_height * self.args.depth_ground_fit_min_v_ratio
        usable = np.flatnonzero(v >= min_v)
        if usable.size == 0:
            usable = np.arange(points.shape[0])

        rows = max(2, self.args.depth_ground_fit_rows)
        edges = np.linspace(float(np.min(v[usable])), float(np.max(v[usable])), rows + 1)
        selected = []
        percentile = min(100.0, max(0.0, self.args.depth_ground_fit_percentile))
        slack = max(0.0, self.args.depth_ground_fit_slack)

        # Pick low-height samples from each image band so far floor can still
        # help the plane fit even when nearby objects occupy the lower image.
        for index in range(rows):
            upper = edges[index]
            lower = edges[index + 1]
            below_upper = v[usable] >= upper
            below_lower = v[usable] <= lower if index == rows - 1 else v[usable] < lower
            in_band = usable[below_upper & below_lower]
            if in_band.size < self.args.depth_ground_fit_min_points_per_row:
                continue
            z_values = points[in_band, 2]
            cutoff = float(np.percentile(z_values, percentile)) + slack
            selected.extend(in_band[z_values <= cutoff])

        if not selected:
            return np.empty(0, dtype=np.int32)
        return np.array(selected, dtype=np.int32)

    def solve_ground_plane(self, points: object) -> object:
        matrix = np.column_stack((points[:, 0], points[:, 1], np.ones(points.shape[0])))
        coeff, *_ = np.linalg.lstsq(matrix, points[:, 2], rcond=None)
        return coeff.astype(np.float32)

    def height_above_ground(self, points: object, plane: object) -> object:
        ground_z = points[:, 0] * plane[0] + points[:, 1] * plane[1] + plane[2]
        return points[:, 2] - ground_z

    def estimate_ground_height(self, samples: DepthPixelMap) -> float:
        points = samples.local_points
        if points.size == 0:
            return 0.0

        # The floor usually appears lower in the image. A low z percentile keeps
        # the estimate from jumping up when a person or robot enters the frame.
        lower_image = samples.pixel_v >= samples.image_height * self.args.depth_ground_image_min_v_ratio
        candidates = points[lower_image] if np.any(lower_image) else points
        percentile = min(100.0, max(0.0, self.args.depth_ground_percentile))
        return float(np.percentile(candidates[:, 2], percentile))

    def project(self, depth_m: object, intrinsics: CameraIntrinsics) -> DepthPixelMap:
        height, width = depth_m.shape
        ys = np.arange(0, height, self.args.depth_sample_step)
        xs = np.arange(0, width, self.args.depth_sample_step)
        grid_x, grid_y = np.meshgrid(xs, ys)
        sampled_depth = depth_m[grid_y, grid_x]

        valid = np.isfinite(sampled_depth)
        valid &= sampled_depth >= self.args.depth_min_depth
        valid &= sampled_depth <= self.args.depth_max_depth
        if not np.any(valid):
            empty = np.empty((0, 3), dtype=np.float32)
            return DepthPixelMap(empty, np.empty(0), np.empty(0), width, height)

        u = grid_x[valid].astype(np.float32)
        v = grid_y[valid].astype(np.float32)
        z_cam = sampled_depth[valid].astype(np.float32)
        x_cam = (u - intrinsics.cx) * z_cam / intrinsics.fx
        y_cam = (v - intrinsics.cy) * z_cam / intrinsics.fy

        ones = np.ones_like(z_cam)
        points_cam = np.vstack((x_cam, y_cam, z_cam, ones))
        points_local = (self.transform @ points_cam)[:3, :].T

        # Keep only the near-field region that matters for the robot's local
        # obstacle awareness, and ignore the robot body directly below camera.
        x = points_local[:, 0]
        y = points_local[:, 1]
        in_map = x >= self.args.depth_map_min_x
        in_map &= x <= self.args.depth_map_max_x
        in_map &= np.abs(y) <= self.args.depth_map_half_width

        self_body = x <= self.args.depth_self_exclusion_x
        self_body &= np.abs(y) <= self.args.depth_self_exclusion_y
        keep = in_map & ~self_body

        return DepthPixelMap(points_local[keep], u[keep], v[keep], width, height)

    def regions_for_mask(
        self,
        kind: str,
        samples: DepthPixelMap,
        mask: object,
        height_above_ground: object,
    ) -> list[DepthRegion]:
        if not np.any(mask):
            return []

        points = samples.local_points[mask]
        heights = height_above_ground[mask]
        u = samples.pixel_u[mask]
        v = samples.pixel_v[mask]
        cols = max(1, self.args.depth_region_cols)
        rows = max(1, self.args.depth_region_rows)
        max_polygons = max(0, self.args.depth_max_polygons_per_class)
        min_points = max(1, self.args.depth_min_region_points)
        pad = max(0, self.args.depth_polygon_padding_px)

        cell_x = np.clip((u / max(1, samples.image_width) * cols).astype(np.int32), 0, cols - 1)
        cell_y = np.clip((v / max(1, samples.image_height) * rows).astype(np.int32), 0, rows - 1)
        cell_id = cell_y * cols + cell_x

        regions = []
        for key in np.unique(cell_id):
            indices = np.flatnonzero(cell_id == key)
            if indices.size < min_points:
                continue

            region_u = u[indices]
            region_v = v[indices]
            region_points = points[indices]
            region_heights = heights[indices]
            xmin = int(max(0, math.floor(float(np.min(region_u)) - pad)))
            ymin = int(max(0, math.floor(float(np.min(region_v)) - pad)))
            xmax = int(min(samples.image_width - 1, math.ceil(float(np.max(region_u)) + pad)))
            ymax = int(min(samples.image_height - 1, math.ceil(float(np.max(region_v)) + pad)))
            if xmax <= xmin or ymax <= ymin:
                continue

            mean_x = float(np.mean(region_points[:, 0]))
            mean_y = float(np.mean(region_points[:, 1]))
            nearest_x = float(np.min(region_points[:, 0]))
            max_height = float(np.max(region_heights))
            front = (
                kind == "obstacle"
                and self.args.depth_front_min_x <= nearest_x <= self.args.depth_front_max_x
                and abs(mean_y) <= self.args.depth_front_half_width
            )
            regions.append(
                DepthRegion(
                    kind=kind,
                    polygon=[[xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax]],
                    count=int(indices.size),
                    mean_x=mean_x,
                    mean_y=mean_y,
                    nearest_x=nearest_x,
                    max_height=max_height,
                    source_width=samples.image_width,
                    source_height=samples.image_height,
                    front=front,
                )
            )

        # Show the strongest evidence first. Obstacles with many points produce
        # larger, more stable red polygons in the overlay.
        regions.sort(key=lambda region: region.count, reverse=True)
        if kind == "obstacle":
            return regions
        return regions[:max_polygons] if max_polygons else regions

    def obstacle_regions_for_components(
        self,
        samples: DepthPixelMap,
        mask: object,
        height_above_ground: object,
    ) -> list[DepthRegion]:
        """Group obstacle points by actual connected image components.

        This avoids the old coarse grid behavior where separate chairs, people,
        and tables could all chain into one huge red rectangle.
        """

        if not np.any(mask):
            return []

        points = samples.local_points[mask]
        heights = height_above_ground[mask]
        u = samples.pixel_u[mask]
        v = samples.pixel_v[mask]
        cell_px = max(1, self.args.depth_obstacle_component_cell_px)
        gap_cells = max(0, self.args.depth_obstacle_component_gap_cells)
        min_points = max(1, self.args.depth_min_region_points)
        min_points_per_cell = max(1, self.args.depth_obstacle_component_min_points_per_cell)
        pad = max(0, self.args.depth_polygon_padding_px)
        cols = max(1, int(math.ceil(samples.image_width / cell_px)))
        rows = max(1, int(math.ceil(samples.image_height / cell_px)))

        cell_x = np.clip((u / cell_px).astype(np.int32), 0, cols - 1)
        cell_y = np.clip((v / cell_px).astype(np.int32), 0, rows - 1)
        occupied = np.zeros((rows, cols), dtype=bool)
        cell_indices: dict[tuple[int, int], list[int]] = {}
        for index, (cx, cy) in enumerate(zip(cell_x, cell_y)):
            key = (int(cy), int(cx))
            cell_indices.setdefault(key, []).append(index)

        cell_mean_x: dict[tuple[int, int], float] = {}
        cell_mean_y: dict[tuple[int, int], float] = {}
        for key, indices_for_cell in cell_indices.items():
            if len(indices_for_cell) < min_points_per_cell:
                continue
            cell_points = points[np.array(indices_for_cell, dtype=np.int32)]
            occupied[key] = True
            cell_mean_x[key] = float(np.mean(cell_points[:, 0]))
            cell_mean_y[key] = float(np.mean(cell_points[:, 1]))

        visited = np.zeros((rows, cols), dtype=bool)
        regions = []
        for start_y, start_x in np.argwhere(occupied):
            start = (int(start_y), int(start_x))
            if visited[start]:
                continue

            stack = [start]
            component_cells = []
            visited[start] = True
            while stack:
                cy, cx = stack.pop()
                component_cells.append((cy, cx))
                for ny in range(max(0, cy - gap_cells - 1), min(rows, cy + gap_cells + 2)):
                    for nx in range(max(0, cx - gap_cells - 1), min(cols, cx + gap_cells + 2)):
                        if visited[ny, nx] or not occupied[ny, nx]:
                            continue
                        if not self.obstacle_cells_are_close(
                            (cy, cx), (int(ny), int(nx)), cell_mean_x, cell_mean_y
                        ):
                            continue
                        visited[ny, nx] = True
                        stack.append((ny, nx))

            indices = []
            for key in component_cells:
                indices.extend(cell_indices.get(key, []))
            indices = np.array(indices, dtype=np.int32)
            if indices.size < min_points:
                continue

            region = self.depth_region_from_indices(
                "obstacle", samples, u, v, points, heights, indices, pad
            )
            if region is not None:
                regions.append(region)

        regions.sort(key=lambda region: region.count, reverse=True)
        max_polygons = max(0, self.args.depth_max_polygons_per_class)
        return regions[:max_polygons] if max_polygons else regions

    def obstacle_cells_are_close(
        self,
        first: tuple[int, int],
        second: tuple[int, int],
        cell_mean_x: dict[tuple[int, int], float],
        cell_mean_y: dict[tuple[int, int], float],
    ) -> bool:
        if abs(cell_mean_x[first] - cell_mean_x[second]) > self.args.depth_obstacle_component_max_x_delta:
            return False
        if abs(cell_mean_y[first] - cell_mean_y[second]) > self.args.depth_obstacle_component_max_y_delta:
            return False
        return True

    def depth_region_from_indices(
        self,
        kind: str,
        samples: DepthPixelMap,
        u: object,
        v: object,
        points: object,
        heights: object,
        indices: object,
        pad: int,
    ) -> Optional[DepthRegion]:
        region_u = u[indices]
        region_v = v[indices]
        region_points = points[indices]
        region_heights = heights[indices]
        xmin = int(max(0, math.floor(float(np.min(region_u)) - pad)))
        ymin = int(max(0, math.floor(float(np.min(region_v)) - pad)))
        xmax = int(min(samples.image_width - 1, math.ceil(float(np.max(region_u)) + pad)))
        ymax = int(min(samples.image_height - 1, math.ceil(float(np.max(region_v)) + pad)))
        if xmax <= xmin or ymax <= ymin:
            return None

        mean_x = float(np.mean(region_points[:, 0]))
        mean_y = float(np.mean(region_points[:, 1]))
        nearest_x = float(np.min(region_points[:, 0]))
        max_height = float(np.max(region_heights))
        front = (
            kind == "obstacle"
            and self.args.depth_front_min_x <= nearest_x <= self.args.depth_front_max_x
            and abs(mean_y) <= self.args.depth_front_half_width
        )

        return DepthRegion(
            kind=kind,
            polygon=[[xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax]],
            count=int(indices.size),
            mean_x=mean_x,
            mean_y=mean_y,
            nearest_x=nearest_x,
            max_height=max_height,
            source_width=samples.image_width,
            source_height=samples.image_height,
            front=front,
        )

    def ground_mesh_for_mask(
        self,
        samples: DepthPixelMap,
        mask: object,
        height_above_ground: object,
    ) -> Optional[DepthRegion]:
        """Create one green ground zone with mesh lines instead of floor boxes."""

        if not np.any(mask):
            return None

        points = samples.local_points[mask]
        heights = height_above_ground[mask]
        u = samples.pixel_u[mask]
        v = samples.pixel_v[mask]
        min_points = max(1, self.args.depth_ground_mesh_min_points_per_row)
        rows = max(2, self.args.depth_ground_mesh_rows)
        edge_percentile = min(45.0, max(0.0, self.args.depth_ground_mesh_edge_percentile))
        min_width = max(1, self.args.depth_ground_mesh_min_width_px)

        v_min = float(np.min(v))
        v_max = float(np.max(v))
        if v_max <= v_min:
            return None

        bands = []
        band_edges = np.linspace(v_min, v_max, rows + 1)
        for index in range(rows):
            upper = band_edges[index]
            lower = band_edges[index + 1]
            in_band = (v >= upper) & (v <= lower if index == rows - 1 else v < lower)
            if int(np.count_nonzero(in_band)) < min_points:
                continue

            band_u = u[in_band]
            band_v = v[in_band]
            left = float(np.percentile(band_u, edge_percentile))
            right = float(np.percentile(band_u, 100.0 - edge_percentile))
            center_v = float(np.median(band_v))
            if right - left < min_width:
                continue
            bands.append((left, right, center_v))

        if len(bands) < 2:
            return None

        # Build a single floor-zone polygon by walking down the left edge and
        # back up the right edge. This reads as a ground mesh, not box clutter.
        bands.sort(key=lambda item: item[2])
        left_edge = [[int(round(left)), int(round(y))] for left, _, y in bands]
        right_edge = [[int(round(right)), int(round(y))] for _, right, y in reversed(bands)]
        polygon = left_edge + right_edge

        mesh_lines = []
        for left, right, y in bands:
            mesh_lines.append([[int(round(left)), int(round(y))], [int(round(right)), int(round(y))]])

        for fraction in self.args.depth_ground_mesh_vertical_fractions:
            column = []
            for left, right, y in bands:
                x = left + (right - left) * fraction
                column.append([int(round(x)), int(round(y))])
            if len(column) >= 2:
                mesh_lines.append(column)

        return DepthRegion(
            kind="flat",
            polygon=polygon,
            count=int(points.shape[0]),
            mean_x=float(np.mean(points[:, 0])),
            mean_y=float(np.mean(points[:, 1])),
            nearest_x=float(np.min(points[:, 0])),
            max_height=float(np.max(heights)),
            source_width=samples.image_width,
            source_height=samples.image_height,
            mesh_lines=mesh_lines,
        )

    def merge_obstacle_regions(self, regions: list[DepthRegion]) -> list[DepthRegion]:
        """Merge obstacle boxes that overlap or sit right next to each other."""

        gap = max(0, self.args.depth_obstacle_merge_gap_px)
        merged = list(regions)
        changed = True
        while changed:
            changed = False
            next_regions = []
            consumed = [False] * len(merged)
            for index, region in enumerate(merged):
                if consumed[index]:
                    continue
                current = region
                consumed[index] = True
                for other_index in range(index + 1, len(merged)):
                    if consumed[other_index]:
                        continue
                    other = merged[other_index]
                    if self.regions_are_close(current, other, gap):
                        current = self.merge_two_regions(current, other)
                        consumed[other_index] = True
                        changed = True
                next_regions.append(current)
            merged = next_regions

        merged.sort(key=lambda region: region.count, reverse=True)
        max_polygons = max(0, self.args.depth_max_polygons_per_class)
        return merged[:max_polygons] if max_polygons else merged

    def regions_are_close(self, first: DepthRegion, second: DepthRegion, gap: int) -> bool:
        if abs(first.mean_x - second.mean_x) > self.args.depth_obstacle_merge_max_x_delta:
            return False
        if abs(first.nearest_x - second.nearest_x) > self.args.depth_obstacle_merge_max_nearest_x_delta:
            return False
        if abs(first.mean_y - second.mean_y) > self.args.depth_obstacle_merge_max_y_delta:
            return False

        first_box = self.expanded_region_box(first)
        second_box = self.expanded_region_box(second)
        boxes_touch = not (
            first_box[2] + gap < second_box[0]
            or second_box[2] + gap < first_box[0]
            or first_box[3] + gap < second_box[1]
            or second_box[3] + gap < first_box[1]
        )
        if not boxes_touch:
            return False

        union_box = (
            min(first_box[0], second_box[0]),
            min(first_box[1], second_box[1]),
            max(first_box[2], second_box[2]),
            max(first_box[3], second_box[3]),
        )
        union_area = self.box_area(union_box)
        separate_area = self.box_area(first_box) + self.box_area(second_box)
        return union_area <= separate_area * self.args.depth_obstacle_merge_max_area_growth

    def expanded_region_box(self, region: DepthRegion, gap: int = 0) -> tuple[int, int, int, int]:
        xs = [point[0] for point in region.polygon]
        ys = [point[1] for point in region.polygon]
        return min(xs) - gap, min(ys) - gap, max(xs) + gap, max(ys) + gap

    def box_area(self, box: tuple[int, int, int, int]) -> int:
        return max(1, box[2] - box[0]) * max(1, box[3] - box[1])

    def merge_two_regions(self, first: DepthRegion, second: DepthRegion) -> DepthRegion:
        first_box = self.expanded_region_box(first)
        second_box = self.expanded_region_box(second)
        xmin = min(first_box[0], second_box[0])
        ymin = min(first_box[1], second_box[1])
        xmax = max(first_box[2], second_box[2])
        ymax = max(first_box[3], second_box[3])
        total_count = max(1, first.count + second.count)

        return DepthRegion(
            kind="obstacle",
            polygon=[[xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax]],
            count=total_count,
            mean_x=(first.mean_x * first.count + second.mean_x * second.count) / total_count,
            mean_y=(first.mean_y * first.count + second.mean_y * second.count) / total_count,
            nearest_x=min(first.nearest_x, second.nearest_x),
            max_height=max(first.max_height, second.max_height),
            source_width=first.source_width,
            source_height=first.source_height,
            front=first.front or second.front,
        )


class DepthOverlayNode(Node):
    """ROS node that feeds depth-derived regions into the web server store."""

    def __init__(self, store: FrameStore, args: argparse.Namespace) -> None:
        super().__init__("booster_camera_depth_overlay")
        self.store = store
        self.args = args
        self.ball_observations: list[BallObservation] = []
        camera_config = load_vision_config(args.depth_vision_config)
        self.intrinsics = intrinsics_from_config(camera_config)

        transform = matrix_from_config(camera_config) if args.depth_use_config_extrin else None
        if transform is None:
            transform = default_optical_to_local_matrix()

        self.builder = DepthOverlayBuilder(args, transform)
        self.create_subscription(
            CameraInfo,
            args.depth_camera_info_topic,
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(Image, args.depth_topic, self.on_depth, qos_profile_sensor_data)

        if args.depth_filter_ball:
            if VisionDetections is not None:
                self.create_subscription(
                    VisionDetections,
                    args.depth_detection_topic,
                    self.on_detections,
                    10,
                )
                self.get_logger().info(f"Depth overlay ball filter listening on {args.depth_detection_topic}")
            else:
                self.get_logger().warning("vision_interface/Detections unavailable; detection ball filter disabled")

            if VisionBall is not None:
                self.create_subscription(
                    VisionBall,
                    args.depth_ball_topic,
                    self.on_ball,
                    10,
                )
                self.get_logger().info(f"Depth overlay ball filter listening on {args.depth_ball_topic}")
            else:
                self.get_logger().warning("vision_interface/Ball unavailable; ball topic filter disabled")

        self.store.set_depth_status(connected=True)
        self.get_logger().info(f"Depth overlay listening on {args.depth_topic}")
        self.get_logger().info(f"Depth overlay camera info on {args.depth_camera_info_topic}")

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.intrinsics = CameraIntrinsics.from_camera_info(msg)

    def on_detections(self, msg) -> None:
        now = time.time()
        draw_detections = []
        for obj in msg.detected_objects:
            xmin = int(obj.xmin)
            ymin = int(obj.ymin)
            xmax = int(obj.xmax)
            ymax = int(obj.ymax)
            if xmax > xmin and ymax > ymin:
                draw_detections.append(
                    Detection(
                        label=str(obj.label),
                        confidence=float(obj.confidence),
                        xmin=xmin,
                        ymin=ymin,
                        xmax=xmax,
                        ymax=ymax,
                        position=[float(value) for value in obj.position],
                        position_projection=[float(value) for value in obj.position_projection],
                    )
                )

            if obj.label.strip().lower() != "ball":
                continue
            if obj.confidence < self.args.depth_ball_filter_min_confidence:
                continue
            if len(obj.position_projection) < 2:
                continue
            self.ball_observations.append(
                BallObservation(
                    x=float(obj.position_projection[0]),
                    y=float(obj.position_projection[1]),
                    confidence=float(obj.confidence),
                    updated_at=now,
                )
            )
        self.prune_ball_observations(now)
        self.store.set_detections(draw_detections)

    def on_ball(self, msg) -> None:
        if msg.confidence < self.args.depth_ball_filter_min_confidence:
            return
        now = time.time()
        self.ball_observations.append(
            BallObservation(
                x=float(msg.x),
                y=float(msg.y),
                confidence=float(msg.confidence),
                updated_at=now,
            )
        )
        self.prune_ball_observations(now)

    def fresh_ball_observations(self) -> list[BallObservation]:
        now = time.time()
        self.prune_ball_observations(now)
        return list(self.ball_observations)

    def prune_ball_observations(self, now: float) -> None:
        cutoff = now - self.args.depth_ball_filter_max_age
        self.ball_observations = [
            observation
            for observation in self.ball_observations
            if observation.updated_at >= cutoff
        ][-self.args.depth_ball_filter_max_observations :]

    def on_depth(self, msg: Image) -> None:
        if self.intrinsics is None:
            self.store.set_depth_status(connected=True, error="waiting for depth CameraInfo")
            return

        try:
            depth_m = image_to_depth_meters(msg)
            regions = self.builder.build_regions(
                depth_m,
                self.intrinsics,
                self.fresh_ball_observations(),
            )
        except Exception as exc:
            self.store.set_depth_status(connected=True, error=str(exc))
            return

        self.store.set_depth_regions(regions)


def depth_overlay_reader(store: FrameStore, args: argparse.Namespace) -> None:
    """Run the optional ROS depth subscriber in a background thread."""

    if np is None or rclpy is None:
        store.set_depth_status(
            connected=False,
            error="depth overlay needs numpy, rclpy, and sensor_msgs on the robot",
        )
        return

    node = None
    try:
        rclpy.init(args=None)
        node = DepthOverlayNode(store, args)
        rclpy.spin(node)
    except Exception as exc:
        store.set_depth_status(connected=False, error=str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


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

            if self.path == "/depth_overlay.json":
                data = json.dumps(store.depth_snapshot()).encode("utf-8")
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
    .ball-depth-text { fill: #7df9ff; font-size: 21px; font-weight: 800; paint-order: stroke; stroke: #000; stroke-width: 4px; stroke-linejoin: round; }
    .flat-poly { fill: rgba(0, 230, 118, .16); stroke: #00e676; stroke-width: 3; vector-effect: non-scaling-stroke; }
    .flat-mesh-line { fill: none; stroke: rgba(210, 255, 225, .85); stroke-width: 2; vector-effect: non-scaling-stroke; }
    .obstacle-poly { fill: rgba(255, 45, 22, .30); stroke: #ff2d16; stroke-width: 4; vector-effect: non-scaling-stroke; }
    .front-obstacle-poly { fill: rgba(255, 45, 22, .42); stroke: #fff; stroke-width: 5; vector-effect: non-scaling-stroke; }
    .depth-text { fill: #fff; font-size: 20px; font-weight: 800; paint-order: stroke; stroke: #000; stroke-width: 4px; stroke-linejoin: round; }
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
    let latestDetections = [];
    let latestDepthRegions = [];

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

    function depthScale(region) {
      const sourceWidth = Number(region.source_width);
      const sourceHeight = Number(region.source_height);
      return {
        x: img.naturalWidth && sourceWidth ? img.naturalWidth / sourceWidth : 1,
        y: img.naturalHeight && sourceHeight ? img.naturalHeight / sourceHeight : 1,
      };
    }

    function scaledDepthPoint(point, region) {
      const scale = depthScale(region);
      return [Number(point[0]) * scale.x, Number(point[1]) * scale.y];
    }

    function polygonPoints(region) {
      return (region.polygon ?? [])
        .map((point) => {
          const [x, y] = scaledDepthPoint(point, region);
          return `${x},${y}`;
        })
        .join(' ');
    }

    function polylinePoints(line, region) {
      return (line ?? [])
        .map((point) => {
          const [x, y] = scaledDepthPoint(point, region);
          return `${x},${y}`;
        })
        .join(' ');
    }

    function detectionIsBall(det) {
      return String(det.label ?? '').trim().toLowerCase() === 'ball';
    }

    function detectionBox(det) {
      return [Number(det.xmin), Number(det.ymin), Number(det.xmax), Number(det.ymax)];
    }

    function depthRegionBox(region) {
      const points = (region.polygon ?? []).map((point) => scaledDepthPoint(point, region));
      const xs = points.map((point) => point[0]);
      const ys = points.map((point) => point[1]);
      return [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)];
    }

    function expandBox(box, fraction) {
      const width = box[2] - box[0];
      const height = box[3] - box[1];
      return [
        box[0] - width * fraction,
        box[1] - height * fraction,
        box[2] + width * fraction,
        box[3] + height * fraction,
      ];
    }

    function boxArea(box) {
      return Math.max(1, box[2] - box[0]) * Math.max(1, box[3] - box[1]);
    }

    function intersectionArea(a, b) {
      const x0 = Math.max(a[0], b[0]);
      const y0 = Math.max(a[1], b[1]);
      const x1 = Math.min(a[2], b[2]);
      const y1 = Math.min(a[3], b[3]);
      if (x1 <= x0 || y1 <= y0) {
        return 0;
      }
      return (x1 - x0) * (y1 - y0);
    }

    function numericVector(value) {
      if (!Array.isArray(value)) {
        return [];
      }
      const values = value.map(Number);
      return values.every(Number.isFinite) ? values : [];
    }

    function vectorHasSignal(values) {
      return values.length >= 2 && values.some((value) => Math.abs(value) > 0.0001);
    }

    function formatMeters(value) {
      return `${value.toFixed(2)}m`;
    }

    function ballPositionLines(det) {
      if (!detectionIsBall(det)) {
        return [];
      }

      const depthPosition = numericVector(det.position);
      const projection = numericVector(det.position_projection);
      const lines = [];

      if (vectorHasSignal(depthPosition)) {
        const x = depthPosition[0] ?? 0;
        const y = depthPosition[1] ?? 0;
        const z = depthPosition[2] ?? 0;
        const range = Math.hypot(x, y, z);
        lines.push(`depth x=${formatMeters(x)} y=${formatMeters(y)} z=${formatMeters(z)} r=${formatMeters(range)}`);
      } else {
        lines.push('depth none');
      }

      if (vectorHasSignal(projection)) {
        const x = projection[0] ?? 0;
        const y = projection[1] ?? 0;
        lines.push(`proj x=${formatMeters(x)} y=${formatMeters(y)}`);
      }

      return lines;
    }

    function depthRegionMatchesBall(region) {
      if (region.kind !== 'obstacle' || !(region.polygon ?? []).length) {
        return false;
      }

      const regionBox = depthRegionBox(region);
      const regionArea = boxArea(regionBox);
      const regionCenterX = (regionBox[0] + regionBox[2]) / 2;
      const regionCenterY = (regionBox[1] + regionBox[3]) / 2;

      return latestDetections.some((det) => {
        if (!detectionIsBall(det)) {
          return false;
        }
        const ballBox = detectionBox(det);
        const overlap = intersectionArea(regionBox, ballBox);
        if (overlap <= 0) {
          return false;
        }
        const ballArea = boxArea(ballBox);
        const paddedBall = expandBox(ballBox, 0.2);
        const centerInsideBall =
          paddedBall[0] <= regionCenterX && regionCenterX <= paddedBall[2] &&
          paddedBall[1] <= regionCenterY && regionCenterY <= paddedBall[3];
        return overlap / regionArea >= 0.25 || (centerInsideBall && regionArea <= ballArea * 4);
      });
    }

    function renderDepthRegions(regions, layer) {
      // The depth layer is drawn first so YOLO boxes remain readable on top.
      for (const region of regions) {
        const points = polygonPoints(region);
        if (!points) {
          continue;
        }

        const isObstacle = region.kind === 'obstacle';
        if (isObstacle && depthRegionMatchesBall(region)) {
          continue;
        }
        const className = isObstacle
          ? (region.front ? 'front-obstacle-poly' : 'obstacle-poly')
          : 'flat-poly';
        layer.appendChild(makeSvg('polygon', {class: className, points}));

        if (!isObstacle) {
          for (const line of region.mesh_lines ?? []) {
            const linePoints = polylinePoints(line, region);
            if (linePoints) {
              layer.appendChild(makeSvg('polyline', {class: 'flat-mesh-line', points: linePoints}));
            }
          }
        }

        if (!isObstacle) {
          continue;
        }

        const firstPoint = scaledDepthPoint(region.polygon?.[0] ?? [0, 0], region);
        const labelX = firstPoint[0] + 4;
        const labelY = Math.max(22, firstPoint[1] - 8);
        const meanX = Number(region.mean_x);
        const meanY = Number(region.mean_y);
        const nearestX = Number(region.nearest_x);
        const maxHeight = Number(region.max_height);
        const label = [
          region.front ? 'FRONT obstacle' : 'obstacle',
          Number.isFinite(meanX) ? `x=${meanX.toFixed(2)}m` : '',
          Number.isFinite(meanY) ? `y=${meanY.toFixed(2)}m` : '',
          Number.isFinite(nearestX) ? `near=${nearestX.toFixed(2)}m` : '',
          Number.isFinite(maxHeight) ? `h=${maxHeight.toFixed(2)}m` : '',
          `n=${region.count ?? 0}`,
        ].filter(Boolean).join(' ');
        layer.appendChild(makeSvg('text', {class: 'depth-text', x: labelX, y: labelY}, label));
        layer.lastChild.textContent = label;
      }
    }

    function renderDetections(detections, layer) {
      // YOLO boxes are still drawn in the original image coordinate space.
      setOverlayViewBox();
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
        const positionLines = ballPositionLines(det);
        const lineGap = 24;
        const infoLines = [`center=(${centerX},${centerY})`, ...positionLines];
        const belowStartY = y + height + 34;
        const aboveStartY = y - 34 - (infoLines.length - 1) * lineGap;
        const infoBlockBottom = belowStartY + (infoLines.length - 1) * lineGap;
        const infoStartY =
          img.naturalHeight && infoBlockBottom > img.naturalHeight
            ? Math.max(24, aboveStartY)
            : belowStartY;
        layer.appendChild(makeSvg('rect', {class: 'box', x, y, width, height, rx: 2}));
        layer.appendChild(makeSvg('text', {class: 'box-text', x: x + 4, y: textY}, label));
        layer.lastChild.textContent = label;
        layer.appendChild(makeSvg('circle', {class: 'center-dot', cx: centerX, cy: centerY, r: 8}));
        infoLines.forEach((line, index) => {
          const className = index === 0 ? 'center-text' : 'ball-depth-text';
          layer.appendChild(makeSvg('text', {class: className, x: x + 4, y: infoStartY + index * lineGap}, line));
          layer.lastChild.textContent = line;
        });
      }
    }

    function renderOverlay() {
      setOverlayViewBox();
      const depthLayer = makeSvg('g', {id: 'depth-overlay-layer'});
      const yoloLayer = makeSvg('g', {id: 'yolo-detection-layer'});
      renderDepthRegions(latestDepthRegions, depthLayer);
      renderDetections(latestDetections, yoloLayer);
      overlay.replaceChildren(depthLayer, yoloLayer);
    }

    async function updateStats() {
      try {
        const res = await fetch('/stats.json', {cache: 'no-store'});
        const stats = await res.json();
        const age = stats.last_frame_age_sec == null ? 'no frame' : `${stats.last_frame_age_sec.toFixed(1)}s`;
        const detAge = stats.detections_last_update_age_sec == null ? 'no detections' : `${stats.detections_last_update_age_sec.toFixed(1)}s`;
        const depthAge = stats.depth_last_update_age_sec == null ? 'no depth' : `${stats.depth_last_update_age_sec.toFixed(1)}s`;
        document.getElementById('stats').textContent =
          `${stats.connected ? 'camera' : 'camera reconnecting'} | frame ${stats.frame_id} | age ${age} | boxes ${stats.detections_count} | depth ${stats.depth_region_count} (${depthAge}) | det ${detAge}`;
      } catch (_) {
        document.getElementById('stats').textContent = 'stats unavailable';
      }
    }

    async function updateDetections() {
      try {
        const res = await fetch('/detections.json', {cache: 'no-store'});
        const payload = await res.json();
        latestDetections = payload.detections ?? [];
      } catch (_) {
        latestDetections = [];
      }
      renderOverlay();
    }

    async function updateDepthOverlay() {
      try {
        const res = await fetch('/depth_overlay.json', {cache: 'no-store'});
        const payload = await res.json();
        latestDepthRegions = payload.regions ?? [];
      } catch (_) {
        latestDepthRegions = [];
      }
      renderOverlay();
    }

    updateStats();
    updateDetections();
    updateDepthOverlay();
    setInterval(updateDetections, 100);
    setInterval(updateDepthOverlay, 100);
    setInterval(updateStats, 1000);
    setInterval(renderOverlay, 1000);
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
    parser.add_argument(
        "--depth-overlay",
        action="store_true",
        help="overlay green flat-surface and red obstacle regions from the depth camera",
    )
    parser.add_argument("--depth-topic", default=DEFAULT_DEPTH_TOPIC, help="ROS depth image topic")
    parser.add_argument(
        "--depth-camera-info-topic",
        default=DEFAULT_DEPTH_CAMERA_INFO_TOPIC,
        help="ROS depth camera info topic",
    )
    parser.add_argument(
        "--depth-vision-config",
        default="src/vision/config/vision.yaml",
        help="vision.yaml path used for fallback intrinsics/extrinsics",
    )
    parser.add_argument("--depth-detection-topic", default="/booster_soccer/detection")
    parser.add_argument("--depth-ball-topic", default="/booster_soccer/ball")
    parser.add_argument(
        "--depth-filter-ball",
        nargs="?",
        const=True,
        default=True,
        type=str_to_bool,
        help="remove depth obstacle regions that match recent vision Ball detections",
    )
    parser.add_argument("--depth-ball-filter-min-confidence", type=float, default=0.35)
    parser.add_argument("--depth-ball-filter-max-age", type=float, default=0.75)
    parser.add_argument("--depth-ball-filter-x-radius", type=float, default=0.40)
    parser.add_argument("--depth-ball-filter-y-radius", type=float, default=0.35)
    parser.add_argument("--depth-ball-filter-max-region-points", type=int, default=450)
    parser.add_argument("--depth-ball-filter-max-observations", type=int, default=8)
    parser.add_argument("--depth-sample-step", type=int, default=8, help="sample every Nth depth pixel")
    parser.add_argument("--depth-min-depth", type=float, default=0.15)
    parser.add_argument("--depth-max-depth", type=float, default=6.0)
    parser.add_argument("--depth-map-min-x", type=float, default=0.0)
    parser.add_argument("--depth-map-max-x", type=float, default=5.0)
    parser.add_argument("--depth-map-half-width", type=float, default=3.0)
    parser.add_argument("--depth-self-exclusion-x", type=float, default=0.20)
    parser.add_argument("--depth-self-exclusion-y", type=float, default=0.35)
    parser.add_argument("--depth-ground-min-height", type=float, default=-0.10)
    parser.add_argument("--depth-ground-max-height", type=float, default=0.14)
    parser.add_argument(
        "--depth-auto-ground",
        nargs="?",
        const=True,
        default=True,
        type=str_to_bool,
        help="estimate floor height from each depth frame before classifying flat/obstacle points",
    )
    parser.add_argument(
        "--depth-ground-percentile",
        type=float,
        default=12.0,
        help="low z percentile used as the visible floor estimate",
    )
    parser.add_argument(
        "--depth-ground-image-min-v-ratio",
        type=float,
        default=0.45,
        help="only use pixels below this image-height ratio when estimating floor height",
    )
    parser.add_argument(
        "--depth-ground-fit-min-v-ratio",
        type=float,
        default=0.18,
        help="minimum image-height ratio used when fitting the tilted floor plane",
    )
    parser.add_argument("--depth-ground-fit-rows", type=int, default=10)
    parser.add_argument("--depth-ground-fit-min-points-per-row", type=int, default=5)
    parser.add_argument(
        "--depth-ground-fit-percentile",
        type=float,
        default=38.0,
        help="low-height percentile selected from each image band for floor-plane fitting",
    )
    parser.add_argument(
        "--depth-ground-fit-slack",
        type=float,
        default=0.04,
        help="extra height margin, in meters, above the selected floor percentile",
    )
    parser.add_argument(
        "--depth-ground-plane-inlier-height",
        type=float,
        default=0.12,
        help="height residual, in meters, used when refining the fitted floor plane",
    )
    parser.add_argument("--depth-ground-plane-refine-iterations", type=int, default=2)
    parser.add_argument("--depth-ground-mesh-rows", type=int, default=8)
    parser.add_argument("--depth-ground-mesh-min-points-per-row", type=int, default=6)
    parser.add_argument("--depth-ground-mesh-edge-percentile", type=float, default=8.0)
    parser.add_argument("--depth-ground-mesh-min-width-px", type=int, default=30)
    parser.add_argument(
        "--depth-ground-mesh-vertical-fractions",
        type=float,
        nargs="*",
        default=[0.25, 0.5, 0.75],
        help="fractions across the floor zone where vertical mesh lines are drawn",
    )
    parser.add_argument("--depth-obstacle-min-height", type=float, default=0.22)
    parser.add_argument("--depth-obstacle-max-height", type=float, default=2.0)
    parser.add_argument("--depth-front-min-x", type=float, default=0.20)
    parser.add_argument("--depth-front-max-x", type=float, default=1.20)
    parser.add_argument("--depth-front-half-width", type=float, default=0.35)
    parser.add_argument("--depth-region-cols", type=int, default=8)
    parser.add_argument("--depth-region-rows", type=int, default=6)
    parser.add_argument("--depth-min-region-points", type=int, default=6)
    parser.add_argument("--depth-max-polygons-per-class", type=int, default=20)
    parser.add_argument("--depth-polygon-padding-px", type=int, default=6)
    parser.add_argument(
        "--depth-obstacle-component-cell-px",
        type=int,
        default=14,
        help="small image cell size used to group obstacle points into connected components",
    )
    parser.add_argument(
        "--depth-obstacle-component-min-points-per-cell",
        type=int,
        default=2,
        help="minimum obstacle samples needed in a component cell",
    )
    parser.add_argument(
        "--depth-obstacle-component-gap-cells",
        type=int,
        default=0,
        help="extra empty component cells allowed when grouping obstacle points",
    )
    parser.add_argument(
        "--depth-obstacle-component-max-x-delta",
        type=float,
        default=0.18,
        help="maximum forward-distance difference, in meters, between connected obstacle cells",
    )
    parser.add_argument(
        "--depth-obstacle-component-max-y-delta",
        type=float,
        default=0.35,
        help="maximum lateral-distance difference, in meters, between connected obstacle cells",
    )
    parser.add_argument(
        "--depth-obstacle-merge-gap-px",
        type=int,
        default=6,
        help="merge obstacle boxes only when their image boxes are this close",
    )
    parser.add_argument(
        "--depth-obstacle-merge-max-x-delta",
        type=float,
        default=0.22,
        help="maximum forward-distance difference, in meters, for merging obstacle boxes",
    )
    parser.add_argument(
        "--depth-obstacle-merge-max-nearest-x-delta",
        type=float,
        default=0.18,
        help="maximum nearest-depth difference, in meters, for merging obstacle boxes",
    )
    parser.add_argument(
        "--depth-obstacle-merge-max-y-delta",
        type=float,
        default=0.35,
        help="maximum lateral-distance difference, in meters, for merging obstacle boxes",
    )
    parser.add_argument(
        "--depth-obstacle-merge-max-area-growth",
        type=float,
        default=1.25,
        help="reject merges that would create a mostly-empty oversized box",
    )
    parser.add_argument(
        "--no-depth-config-extrin",
        action="store_false",
        dest="depth_use_config_extrin",
        help="ignore vision.yaml camera.extrin and use a simple optical-frame conversion",
    )
    parser.set_defaults(depth_use_config_extrin=True)
    args, extras = parser.parse_known_args()
    apply_legacy_key_value_args(args, extras)
    if args.depth_sample_step < 1:
        raise SystemExit("--depth-sample-step must be >= 1")
    if args.depth_ground_mesh_rows < 2:
        raise SystemExit("--depth-ground-mesh-rows must be >= 2")
    if args.depth_ground_mesh_min_points_per_row < 1:
        raise SystemExit("--depth-ground-mesh-min-points-per-row must be >= 1")
    if args.depth_ground_fit_rows < 2:
        raise SystemExit("--depth-ground-fit-rows must be >= 2")
    if args.depth_ground_fit_min_points_per_row < 1:
        raise SystemExit("--depth-ground-fit-min-points-per-row must be >= 1")
    if args.depth_obstacle_component_cell_px < 1:
        raise SystemExit("--depth-obstacle-component-cell-px must be >= 1")
    if args.depth_obstacle_component_min_points_per_cell < 1:
        raise SystemExit("--depth-obstacle-component-min-points-per-cell must be >= 1")
    if args.depth_obstacle_component_gap_cells < 0:
        raise SystemExit("--depth-obstacle-component-gap-cells must be >= 0")
    if args.depth_ball_filter_max_observations < 1:
        raise SystemExit("--depth-ball-filter-max-observations must be >= 1")
    if args.depth_ball_filter_max_region_points < 1:
        raise SystemExit("--depth-ball-filter-max-region-points must be >= 1")
    args.depth_ground_image_min_v_ratio = min(1.0, max(0.0, args.depth_ground_image_min_v_ratio))
    args.depth_ground_fit_min_v_ratio = min(1.0, max(0.0, args.depth_ground_fit_min_v_ratio))
    args.depth_ground_fit_percentile = min(100.0, max(0.0, args.depth_ground_fit_percentile))
    args.depth_ground_mesh_vertical_fractions = [
        min(1.0, max(0.0, fraction))
        for fraction in args.depth_ground_mesh_vertical_fractions
    ]
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

    if args.depth_overlay:
        depth_overlay = threading.Thread(
            target=depth_overlay_reader,
            args=(store, args),
            daemon=True,
        )
        depth_overlay.start()
        print(f"Depth overlay source: {args.depth_topic}", flush=True)

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
