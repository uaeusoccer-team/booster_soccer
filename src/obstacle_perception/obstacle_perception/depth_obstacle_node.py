"""Publish a compact robot-relative obstacle clearance scan from head depth.

The node deliberately owns perception only. It never publishes motion commands;
the brain behavior tree remains the single owner of robot velocity.
"""

from dataclasses import dataclass
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header
from vision_interface.msg import Detections, ObstacleState
import yaml

try:
    from sensor_msgs_py import point_cloud2
except ImportError:  # Optional debug output; obstacle state still works.
    point_cloud2 = None


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass
class BallObservation:
    x: float
    y: float
    updated_at: float


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_matrix(msg: Pose) -> np.ndarray:
    """Return head-to-robot homogeneous transform from a ROS pose."""
    x = float(msg.orientation.x)
    y = float(msg.orientation.y)
    z = float(msg.orientation.z)
    w = float(msg.orientation.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-9:
        rotation = np.eye(3, dtype=np.float32)
    else:
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        rotation = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float32,
        )
    transform = np.eye(4, dtype=np.float32)
    transform[:3, :3] = rotation
    transform[:3, 3] = [msg.position.x, msg.position.y, msg.position.z]
    return transform


def decode_depth(msg: Image) -> np.ndarray:
    encoding = msg.encoding.lower()
    if encoding in ("16uc1", "mono16"):
        dtype = ">u2" if msg.is_bigendian else "<u2"
        width_with_padding = msg.step // np.dtype(dtype).itemsize
        raw = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, width_with_padding)
        return raw[:, : msg.width].astype(np.float32) / 1000.0
    if encoding == "32fc1":
        dtype = ">f4" if msg.is_bigendian else "<f4"
        width_with_padding = msg.step // np.dtype(dtype).itemsize
        raw = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, width_with_padding)
        return raw[:, : msg.width].astype(np.float32)
    raise ValueError(f"unsupported depth encoding: {msg.encoding}")


def load_camera_to_head(config_override: str):
    if config_override:
        config_path = config_override
    else:
        config_path = os.path.join(
            get_package_share_directory("vision"), "config", "vision.yaml"
        )
    with open(config_path, "r", encoding="utf-8") as config_file:
        camera = (yaml.safe_load(config_file) or {}).get("camera", {})
    matrix = np.asarray(camera.get("extrin", []), dtype=np.float32)
    if matrix.shape != (4, 4):
        raise RuntimeError(f"camera.extrin in {config_path} must be a 4x4 matrix")

    rotation = matrix[:3, :3]
    determinant = float(np.linalg.det(rotation))
    orthogonality_error = float(np.linalg.norm(rotation.T @ rotation - np.eye(3)))
    repaired = determinant < 0.5 or orthogonality_error > 0.1
    if repaired:
        # The historical T1 matrix has reliable camera-right and camera-down
        # axes, but its forward column is reflected. Preserve the calibrated
        # axes/translation and reconstruct a proper right-handed rotation.
        camera_right = rotation[:, 0]
        camera_right /= np.linalg.norm(camera_right)
        camera_down = rotation[:, 1] - camera_right * np.dot(camera_right, rotation[:, 1])
        camera_down /= np.linalg.norm(camera_down)
        camera_forward = np.cross(camera_right, camera_down)
        matrix = matrix.copy()
        matrix[:3, :3] = np.column_stack(
            (camera_right, camera_down, camera_forward)
        )
    return matrix, repaired, determinant, orthogonality_error


class DepthObstacleNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_obstacle_node")
        self._declare_parameters()
        (
            self.camera_to_head,
            repaired_transform,
            original_determinant,
            original_orthogonality_error,
        ) = load_camera_to_head(self._param("vision_config_path"))
        if repaired_transform:
            self.get_logger().warning(
                "Repaired non-rigid camera extrinsic for obstacle projection "
                f"(det={original_determinant:.3f}, "
                f"orthogonality_error={original_orthogonality_error:.3f})"
            )
        self.intrinsics = None
        self.head_to_robot = None
        self.head_pose_updated_at = 0.0
        self.ball_observations = []
        self.last_process_time = 0.0
        self.last_status_log = 0.0

        self.directions = np.linspace(
            float(self._param("scan_min_angle")),
            float(self._param("scan_max_angle")),
            int(self._param("scan_samples")),
            dtype=np.float32,
        )

        self.state_pub = self.create_publisher(
            ObstacleState, self._param("obstacle_state_topic"), 10
        )
        self.cloud_pub = None
        if self._param("publish_point_cloud"):
            if point_cloud2 is None:
                self.get_logger().warning(
                    "sensor_msgs_py is unavailable; obstacle point cloud is disabled"
                )
            else:
                self.cloud_pub = self.create_publisher(
                    PointCloud2, self._param("point_cloud_topic"), 5
                )

        self.create_subscription(
            CameraInfo,
            self._param("camera_info_topic"),
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Pose, self._param("head_pose_topic"), self._on_head_pose, 10
        )
        self.create_subscription(
            Detections, self._param("detection_topic"), self._on_detections, 10
        )
        self.create_subscription(
            Image,
            self._param("depth_topic"),
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Depth obstacle perception: {self._param('depth_topic')} -> "
            f"{self._param('obstacle_state_topic')}"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "vision_config_path": "",
            "depth_topic": "/boostercamera/head/depth",
            "camera_info_topic": "/boostercamera/head/depth/camera_info",
            "detection_topic": "/booster_soccer/detection",
            "head_pose_topic": "/head_pose",
            "obstacle_state_topic": "/booster_soccer/obstacle_state",
            "processing_rate_hz": 12.0,
            "sample_step": 8,
            "min_depth": 0.15,
            "max_depth": 3.0,
            "map_min_x": 0.10,
            "map_max_x": 3.0,
            "map_half_width": 2.5,
            "self_exclusion_x": 0.35,
            "self_exclusion_y": 0.45,
            "obstacle_min_height": 0.12,
            "obstacle_max_height": 2.0,
            "scan_min_angle": -1.20,
            "scan_max_angle": 1.20,
            "scan_samples": 17,
            "corridor_half_width": 0.28,
            "minimum_obstacle_points": 8,
            "clearance_quantile": 0.10,
            "blocked_distance": 1.20,
            "ball_min_confidence": 30.0,
            "ball_max_age_sec": 0.75,
            "ball_exclusion_x": 0.35,
            "ball_exclusion_y": 0.30,
            "ball_exclusion_height": 0.40,
            "head_pose_timeout_sec": 0.75,
            "publish_point_cloud": False,
            "point_cloud_topic": "/booster_soccer/local_obstacle_points",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _param(self, name):
        return self.get_parameter(name).value

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if msg.k[0] <= 0.0 or msg.k[4] <= 0.0:
            return
        self.intrinsics = CameraIntrinsics(msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _on_head_pose(self, msg: Pose) -> None:
        self.head_to_robot = quaternion_matrix(msg)
        self.head_pose_updated_at = time.monotonic()

    def _on_detections(self, msg: Detections) -> None:
        now = time.monotonic()
        minimum_confidence = float(self._param("ball_min_confidence"))
        for detected in msg.detected_objects:
            if detected.label.strip().lower() != "ball":
                continue
            if detected.confidence < minimum_confidence:
                continue
            position = None
            if detected.position_confidence > 0 and len(detected.position) >= 2:
                position = detected.position
            elif len(detected.position_projection) >= 2:
                position = detected.position_projection
            if position is None or not np.isfinite(position[:2]).all():
                continue
            self.ball_observations.append(
                BallObservation(float(position[0]), float(position[1]), now)
            )
        self._prune_balls(now)

    def _prune_balls(self, now: float) -> None:
        cutoff = now - float(self._param("ball_max_age_sec"))
        self.ball_observations = [
            ball for ball in self.ball_observations if ball.updated_at >= cutoff
        ][-8:]

    def _on_depth(self, msg: Image) -> None:
        now = time.monotonic()
        rate = max(1.0, float(self._param("processing_rate_hz")))
        if now - self.last_process_time < 1.0 / rate:
            return
        self.last_process_time = now

        if self.intrinsics is None:
            self._publish_invalid(msg.header, "waiting for depth CameraInfo")
            return
        if self.head_to_robot is None or now - self.head_pose_updated_at > float(
            self._param("head_pose_timeout_sec")
        ):
            self._publish_invalid(msg.header, "waiting for fresh head pose")
            return

        try:
            depth = decode_depth(msg)
            points, observed = self._project_obstacles(depth)
            points = self._remove_ball(points, now)
            clearances, counts = self._scan(points, observed)
            self._publish_state(msg.header, points, observed, clearances, counts)
        except (ValueError, RuntimeError) as error:
            self.get_logger().error(str(error))
            self._publish_invalid(msg.header, "depth processing error")

    def _project_obstacles(self, depth: np.ndarray):
        step = max(1, int(self._param("sample_step")))
        rows = np.arange(0, depth.shape[0], step)
        cols = np.arange(0, depth.shape[1], step)
        uu, vv = np.meshgrid(cols, rows)
        sampled_depth = depth[vv, uu]
        valid = np.isfinite(sampled_depth)
        valid &= sampled_depth >= float(self._param("min_depth"))
        valid &= sampled_depth <= float(self._param("max_depth"))

        z = sampled_depth[valid]
        u = uu[valid].astype(np.float32)
        v = vv[valid].astype(np.float32)
        camera_points = np.column_stack(
            (
                (u - self.intrinsics.cx) * z / self.intrinsics.fx,
                (v - self.intrinsics.cy) * z / self.intrinsics.fy,
                z,
                np.ones_like(z),
            )
        )
        camera_to_robot = self.head_to_robot @ self.camera_to_head
        robot_points = (camera_to_robot @ camera_points.T).T[:, :3]

        in_map = robot_points[:, 0] >= float(self._param("map_min_x"))
        in_map &= robot_points[:, 0] <= float(self._param("map_max_x"))
        in_map &= np.abs(robot_points[:, 1]) <= float(self._param("map_half_width"))
        in_map &= robot_points[:, 2] >= float(self._param("obstacle_min_height"))
        in_map &= robot_points[:, 2] <= float(self._param("obstacle_max_height"))
        is_self = robot_points[:, 0] <= float(self._param("self_exclusion_x"))
        is_self &= np.abs(robot_points[:, 1]) <= float(self._param("self_exclusion_y"))
        robot_points = robot_points[in_map & ~is_self]
        return robot_points, self._observed_directions(camera_to_robot, depth.shape[1])

    def _observed_directions(self, camera_to_robot: np.ndarray, width: int) -> np.ndarray:
        ray_pixels = np.array([0.0, self.intrinsics.cx, float(width - 1)])
        rays = np.column_stack(
            (
                (ray_pixels - self.intrinsics.cx) / self.intrinsics.fx,
                np.zeros(3),
                np.ones(3),
            )
        )
        rays_robot = (camera_to_robot[:3, :3] @ rays.T).T
        angles = np.arctan2(rays_robot[:, 1], rays_robot[:, 0])
        center = float(angles[1])
        half_width = max(abs(wrap_angle(float(angle) - center)) for angle in angles)
        return np.array(
            [abs(wrap_angle(float(direction) - center)) <= half_width for direction in self.directions],
            dtype=bool,
        )

    def _remove_ball(self, points: np.ndarray, now: float) -> np.ndarray:
        self._prune_balls(now)
        if not self.ball_observations or points.size == 0:
            return points
        keep = np.ones(points.shape[0], dtype=bool)
        for ball in self.ball_observations:
            matches = np.abs(points[:, 0] - ball.x) <= float(self._param("ball_exclusion_x"))
            matches &= np.abs(points[:, 1] - ball.y) <= float(self._param("ball_exclusion_y"))
            matches &= points[:, 2] <= float(self._param("ball_exclusion_height"))
            keep &= ~matches
        return points[keep]

    def _scan(self, points: np.ndarray, observed: np.ndarray):
        max_range = float(self._param("map_max_x"))
        corridor = float(self._param("corridor_half_width"))
        minimum_points = int(self._param("minimum_obstacle_points"))
        quantile = float(np.clip(self._param("clearance_quantile"), 0.0, 0.5))
        clearances = np.zeros(len(self.directions), dtype=np.float32)
        counts = np.zeros(len(self.directions), dtype=np.uint32)
        if points.size == 0:
            clearances[observed] = max_range
            return clearances, counts

        for index, direction in enumerate(self.directions):
            if not observed[index]:
                continue
            cosine = math.cos(float(direction))
            sine = math.sin(float(direction))
            along = points[:, 0] * cosine + points[:, 1] * sine
            lateral = -points[:, 0] * sine + points[:, 1] * cosine
            in_corridor = along > float(self._param("map_min_x"))
            in_corridor &= along <= max_range
            in_corridor &= np.abs(lateral) <= corridor
            corridor_points = along[in_corridor]
            counts[index] = corridor_points.size
            if corridor_points.size >= minimum_points:
                clearances[index] = float(np.quantile(corridor_points, quantile))
            else:
                clearances[index] = max_range
        return clearances, counts

    def _publish_state(self, header, points, observed, clearances, counts) -> None:
        state = ObstacleState()
        state.header = header
        state.header.frame_id = "base"
        state.valid = True
        state.directions = self.directions.tolist()
        state.clearances = clearances.tolist()
        state.point_counts = counts.tolist()
        state.observed = observed.tolist()
        state.max_range = float(self._param("map_max_x"))

        observed_clearances = clearances[observed]
        state.nearest_distance = (
            float(np.min(observed_clearances)) if observed_clearances.size else 0.0
        )
        center_index = int(np.argmin(np.abs(self.directions)))
        state.blocked = bool(
            not observed[center_index]
            or clearances[center_index] < float(self._param("blocked_distance"))
        )

        safe_distance = float(self._param("blocked_distance"))
        candidates = np.flatnonzero(observed & (clearances >= safe_distance))
        if candidates.size:
            best = min(candidates, key=lambda index: abs(float(self.directions[index])))
        else:
            candidates = np.flatnonzero(observed)
            best = int(candidates[np.argmax(clearances[candidates])]) if candidates.size else center_index
        state.recommended_direction = float(self.directions[best])
        self.state_pub.publish(state)

        if self.cloud_pub is not None:
            cloud_header = Header()
            cloud_header.stamp = header.stamp
            cloud_header.frame_id = "base"
            cloud = point_cloud2.create_cloud_xyz32(
                cloud_header, [tuple(map(float, point)) for point in points]
            )
            self.cloud_pub.publish(cloud)

        now = time.monotonic()
        if now - self.last_status_log >= 1.0:
            self.last_status_log = now
            self.get_logger().info(
                f"valid points={len(points)} nearest={state.nearest_distance:.2f}m "
                f"blocked={state.blocked} steer={state.recommended_direction:+.2f}rad"
            )

    def _publish_invalid(self, header, reason: str) -> None:
        state = ObstacleState()
        state.header = header
        state.header.frame_id = "base"
        state.valid = False
        state.blocked = True
        state.directions = self.directions.tolist()
        state.clearances = [0.0] * len(self.directions)
        state.point_counts = [0] * len(self.directions)
        state.observed = [False] * len(self.directions)
        state.nearest_distance = 0.0
        state.recommended_direction = 0.0
        state.max_range = float(self._param("map_max_x"))
        self.state_pub.publish(state)
        now = time.monotonic()
        if now - self.last_status_log >= 1.0:
            self.last_status_log = now
            self.get_logger().warning(f"Obstacle perception invalid: {reason}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
