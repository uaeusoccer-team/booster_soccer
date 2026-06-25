#!/usr/bin/env python3
"""Passive depth-camera obstacle monitor for the Booster soccer robot.

This script is intentionally standalone: it does not modify or command the
Booster soccer brain, RobotClient, behavior trees, or locomotion stack. It reads
depth-camera ROS topics, projects sampled depth pixels into local 3D points,
and prints a terminal warning when enough elevated points appear in front of
the robot.

Coordinate convention used internally:
    x: forward from the camera/robot
    y: left
    z: up

The local map is only the current camera view, not persistent SLAM. It is a
small foundation that can later feed a visualization overlay or a larger map.
"""

from __future__ import annotations

import argparse
import math
import os
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header

try:
    from sensor_msgs_py import point_cloud2
except ImportError:  # Keep terminal detection usable even if point cloud helpers are absent.
    point_cloud2 = None

try:
    import yaml
except ImportError:
    yaml = None


DEFAULT_DEPTH_TOPIC = "/boostercamera/head/depth"
DEFAULT_CAMERA_INFO_TOPIC = "/boostercamera/head/depth/camera_info"


@dataclass
class CameraIntrinsics:
    """Pinhole camera intrinsics used to turn pixels into 3D rays."""

    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_camera_info(cls, msg: CameraInfo) -> "CameraIntrinsics":
        return cls(fx=msg.k[0], fy=msg.k[4], cx=msg.k[2], cy=msg.k[5])


@dataclass
class LocalMap:
    """Projected points from one depth frame, already filtered to useful range."""

    points: np.ndarray
    flat_points: np.ndarray
    obstacle_points: np.ndarray


@dataclass
class ObstacleReport:
    """Summary of obstacle evidence in the front detection zone."""

    detected: bool
    count: int
    nearest_x: float
    median_y: float
    max_height: float


def load_vision_config(path: str) -> dict:
    """Load camera defaults from src/vision/config/vision.yaml when available."""

    if not path or not os.path.exists(path) or yaml is None:
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    return data.get("camera", {}) or {}


def matrix_from_config(camera_config: dict) -> Optional[np.ndarray]:
    """Return the camera extrinsic matrix from vision.yaml, if present."""

    extrin = camera_config.get("extrin")
    if not extrin:
        return None
    matrix = np.array(extrin, dtype=np.float32)
    if matrix.shape != (4, 4):
        return None
    return matrix


def default_optical_to_local_matrix() -> np.ndarray:
    """Fallback transform: ROS optical frame -> local forward/left/up frame.

    Camera optical frame is normally x-right, y-down, z-forward. For obstacle
    checks, a local robot-like frame is easier to reason about:
        local x = optical z
        local y = -optical x
        local z = -optical y
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


def intrinsics_from_config(camera_config: dict) -> Optional[CameraIntrinsics]:
    """Read fallback intrinsics from vision.yaml if CameraInfo is not available."""

    intrin = camera_config.get("intrin") or {}
    required = ("fx", "fy", "cx", "cy")
    if not all(name in intrin for name in required):
        return None
    return CameraIntrinsics(
        fx=float(intrin["fx"]),
        fy=float(intrin["fy"]),
        cx=float(intrin["cx"]),
        cy=float(intrin["cy"]),
    )


def image_to_depth_meters(msg: Image) -> np.ndarray:
    """Decode ROS depth image data into a float32 array in meters."""

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


class DepthProjector:
    """Projects sampled depth pixels into local points and classifies them."""

    def __init__(self, args: argparse.Namespace, transform: np.ndarray) -> None:
        self.args = args
        self.transform = transform

    def project(self, depth_m: np.ndarray, intrinsics: CameraIntrinsics) -> LocalMap:
        height, width = depth_m.shape
        ys = np.arange(0, height, self.args.sample_step)
        xs = np.arange(0, width, self.args.sample_step)
        grid_x, grid_y = np.meshgrid(xs, ys)
        sampled_depth = depth_m[grid_y, grid_x]

        valid = np.isfinite(sampled_depth)
        valid &= sampled_depth >= self.args.min_depth
        valid &= sampled_depth <= self.args.max_depth
        if not np.any(valid):
            empty = np.empty((0, 3), dtype=np.float32)
            return LocalMap(points=empty, flat_points=empty, obstacle_points=empty)

        u = grid_x[valid].astype(np.float32)
        v = grid_y[valid].astype(np.float32)
        z_cam = sampled_depth[valid].astype(np.float32)
        x_cam = (u - intrinsics.cx) * z_cam / intrinsics.fx
        y_cam = (v - intrinsics.cy) * z_cam / intrinsics.fy

        ones = np.ones_like(z_cam)
        points_cam = np.vstack((x_cam, y_cam, z_cam, ones))
        points_local = (self.transform @ points_cam)[:3, :].T

        # Keep only the local area that is useful for near-field mapping.
        x = points_local[:, 0]
        y = points_local[:, 1]
        z = points_local[:, 2]
        in_map = x >= self.args.map_min_x
        in_map &= x <= self.args.map_max_x
        in_map &= np.abs(y) <= self.args.map_half_width

        # Drop the robot's own chest/body region near the camera.
        self_body = x <= self.args.self_exclusion_x
        self_body &= np.abs(y) <= self.args.self_exclusion_y
        points = points_local[in_map & ~self_body]

        if points.size == 0:
            empty = np.empty((0, 3), dtype=np.float32)
            return LocalMap(points=empty, flat_points=empty, obstacle_points=empty)

        # Flat and obstacle labels are deliberately simple thresholds for now.
        # Tune these on the robot after watching the terminal counts.
        flat_mask = points[:, 2] >= self.args.ground_min_height
        flat_mask &= points[:, 2] <= self.args.ground_max_height

        obstacle_mask = points[:, 2] >= self.args.obstacle_min_height
        obstacle_mask &= points[:, 2] <= self.args.obstacle_max_height

        return LocalMap(
            points=points,
            flat_points=points[flat_mask],
            obstacle_points=points[obstacle_mask],
        )


class FrontObstacleDetector:
    """Checks whether obstacle points are concentrated in front of the robot."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args

    def detect(self, obstacle_points: np.ndarray) -> ObstacleReport:
        if obstacle_points.size == 0:
            return ObstacleReport(False, 0, math.inf, 0.0, 0.0)

        x = obstacle_points[:, 0]
        y = obstacle_points[:, 1]
        z = obstacle_points[:, 2]
        front = x >= self.args.front_min_x
        front &= x <= self.args.front_max_x
        front &= np.abs(y) <= self.args.front_half_width
        front_points = obstacle_points[front]

        count = int(front_points.shape[0])
        if count == 0:
            return ObstacleReport(False, 0, math.inf, 0.0, 0.0)

        return ObstacleReport(
            detected=count >= self.args.min_obstacle_points,
            count=count,
            nearest_x=float(np.min(front_points[:, 0])),
            median_y=float(np.median(front_points[:, 1])),
            max_height=float(np.max(front_points[:, 2])),
        )


class DepthObstacleMonitor(Node):
    """ROS node wrapper that subscribes, maps, detects, prints, and publishes."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("depth_obstacle_monitor")
        self.args = args
        camera_config = load_vision_config(args.vision_config)

        self.intrinsics = intrinsics_from_config(camera_config)
        if self.intrinsics is not None:
            self.get_logger().info("Using fallback intrinsics from vision.yaml until CameraInfo arrives")

        transform = matrix_from_config(camera_config) if args.use_config_extrin else None
        if transform is None:
            transform = default_optical_to_local_matrix()
            self.get_logger().info("Using fallback optical-frame to local-frame transform")
        else:
            self.get_logger().info("Using camera extrinsic matrix from vision.yaml")

        self.projector = DepthProjector(args, transform)
        self.detector = FrontObstacleDetector(args)
        self.last_print_time = 0.0
        self.last_detection_state = False

        self.create_subscription(
            CameraInfo,
            args.camera_info_topic,
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            args.depth_topic,
            self.on_depth,
            qos_profile_sensor_data,
        )

        self.point_pub = None
        self.obstacle_point_pub = None
        if args.publish_point_cloud:
            if point_cloud2 is None:
                self.get_logger().warning("sensor_msgs_py is unavailable; point cloud publishing disabled")
            else:
                self.point_pub = self.create_publisher(PointCloud2, args.point_cloud_topic, 10)
                self.obstacle_point_pub = self.create_publisher(PointCloud2, args.obstacle_point_cloud_topic, 10)

        self.get_logger().info(f"Listening for depth on {args.depth_topic}")
        self.get_logger().info(f"Listening for camera info on {args.camera_info_topic}")

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.intrinsics = CameraIntrinsics.from_camera_info(msg)

    def on_depth(self, msg: Image) -> None:
        if self.intrinsics is None:
            self.get_logger().warning("No CameraInfo/intrinsics yet; skipping depth frame")
            return

        try:
            depth_m = image_to_depth_meters(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return

        local_map = self.projector.project(depth_m, self.intrinsics)
        report = self.detector.detect(local_map.obstacle_points)

        if self.point_pub is not None:
            self.publish_points(self.point_pub, msg.header.stamp, local_map.points)
            self.publish_points(self.obstacle_point_pub, msg.header.stamp, local_map.obstacle_points)

        now = time.time()
        should_print = now - self.last_print_time >= self.args.print_interval
        should_print |= report.detected != self.last_detection_state

        if should_print:
            self.last_print_time = now
            self.last_detection_state = report.detected
            self.print_report(local_map, report)

    def publish_points(self, publisher, stamp, points: np.ndarray) -> None:
        # Use the incoming depth timestamp while keeping the frame explicit for
        # downstream visualization tools.
        header = Header()
        header.stamp = stamp
        header.frame_id = self.args.frame_id
        tuples = [tuple(map(float, point)) for point in points]
        publisher.publish(point_cloud2.create_cloud_xyz32(header, tuples))

    def print_report(self, local_map: LocalMap, report: ObstacleReport) -> None:
        if report.detected:
            self.get_logger().warn(
                "OBSTACLE DETECTED IN FRONT "
                f"points={report.count} nearest_x={report.nearest_x:.2f}m "
                f"median_y={report.median_y:.2f}m max_height={report.max_height:.2f}m"
            )
            return

        self.get_logger().info(
            "No front obstacle "
            f"points={len(local_map.points)} flat={len(local_map.flat_points)} "
            f"obstacle_like={len(local_map.obstacle_points)} front_points={report.count}"
        )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Monitor T1 depth camera for nearby obstacles.")
    parser.add_argument("--vision-config", default="src/vision/config/vision.yaml")
    parser.add_argument("--depth-topic", default=DEFAULT_DEPTH_TOPIC)
    parser.add_argument("--camera-info-topic", default=DEFAULT_CAMERA_INFO_TOPIC)
    parser.add_argument("--sample-step", type=int, default=8, help="Use every Nth pixel in x/y for speed.")
    parser.add_argument("--min-depth", type=float, default=0.15)
    parser.add_argument("--max-depth", type=float, default=4.0)
    parser.add_argument("--map-min-x", type=float, default=0.0)
    parser.add_argument("--map-max-x", type=float, default=3.0)
    parser.add_argument("--map-half-width", type=float, default=2.0)
    parser.add_argument("--self-exclusion-x", type=float, default=0.20)
    parser.add_argument("--self-exclusion-y", type=float, default=0.35)
    parser.add_argument("--ground-min-height", type=float, default=-0.08)
    parser.add_argument("--ground-max-height", type=float, default=0.08)
    parser.add_argument("--obstacle-min-height", type=float, default=0.15)
    parser.add_argument("--obstacle-max-height", type=float, default=2.0)
    parser.add_argument("--front-min-x", type=float, default=0.20)
    parser.add_argument("--front-max-x", type=float, default=1.20)
    parser.add_argument("--front-half-width", type=float, default=0.35)
    parser.add_argument("--min-obstacle-points", type=int, default=30)
    parser.add_argument("--print-interval", type=float, default=0.5)
    parser.add_argument("--frame-id", default="head")
    parser.add_argument("--publish-point-cloud", action="store_true")
    parser.add_argument("--point-cloud-topic", default="/booster_soccer/local_depth_points")
    parser.add_argument("--obstacle-point-cloud-topic", default="/booster_soccer/local_obstacle_points")
    parser.add_argument(
        "--no-config-extrin",
        action="store_false",
        dest="use_config_extrin",
        help="Ignore vision.yaml camera.extrin and use a simple optical-frame conversion.",
    )
    parser.set_defaults(use_config_extrin=True)
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    if args.sample_step < 1:
        raise SystemExit("--sample-step must be >= 1")

    rclpy.init()
    node = DepthObstacleMonitor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
