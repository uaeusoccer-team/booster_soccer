#!/usr/bin/env python3
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from booster_interface.msg import LowState
from booster_msgs.msg import RpcReqMsg
from sensor_msgs.msg import CameraInfo
from vision_interface.msg import Detections


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def valid_number(value: float) -> bool:
    return math.isfinite(float(value))


@dataclass
class BallObservation:
    confidence: float
    cx: float
    cy: float
    width: float
    height: float


class DirectBallHeadTracker(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("direct_ball_head_tracker")
        self.args = args
        self.publisher = self.create_publisher(RpcReqMsg, args.loco_topic, 10)
        self.create_subscription(Detections, args.detection_topic, self.on_detections, 10)
        self.create_subscription(LowState, args.low_state_topic, self.on_low_state, 10)
        if args.camera_info_topic:
            self.create_subscription(CameraInfo, args.camera_info_topic, self.on_camera_info, 10)

        self.latest_ball: Optional[BallObservation] = None
        self.latest_ball_time = 0.0
        self.head_pitch: Optional[float] = None
        self.head_yaw: Optional[float] = None
        self.command_pitch = args.initial_pitch
        self.command_yaw = args.initial_yaw
        self.image_width = float(args.image_width)
        self.image_height = float(args.image_height)
        self.fov_x = math.radians(args.fov_x_deg)
        self.fov_y = math.radians(args.fov_y_deg)
        self.last_command_time = 0.0
        self.last_print_time = 0.0
        self.was_tracking = False
        self.filtered_ball_vx = 0.0
        self.filtered_ball_vy = 0.0
        self.prev_yaw_error: Optional[float] = None
        self.prev_pitch_error: Optional[float] = None
        self.prev_error_time = 0.0
        self.filtered_yaw_error_rate = 0.0
        self.filtered_pitch_error_rate = 0.0

        period = 1.0 / max(args.command_hz, 1.0)
        self.create_timer(period, self.tick)

        print("---- direct ball head tracker ----", flush=True)
        print("No brain behavior tree, no body velocity, no search sweep.", flush=True)
        print(f"detection_topic={args.detection_topic}", flush=True)
        print(f"loco_topic={args.loco_topic} api_id={args.absolute_api_id}", flush=True)
        print(
            f"image={self.image_width:.0f}x{self.image_height:.0f} "
            f"fov=({math.degrees(self.fov_x):.1f}, {math.degrees(self.fov_y):.1f}) deg",
            flush=True,
        )
        print(
            f"deadband={args.deadband_px:.1f}px max_step={args.max_step:.3f}rad "
            f"kp=({args.yaw_kp:.2f}, {args.pitch_kp:.2f}) "
            f"kd=({args.yaw_kd:.2f}, {args.pitch_kd:.2f})",
            flush=True,
        )
        print(
            f"lead_time={args.lead_time:.3f}s max_prediction={args.max_prediction:.3f}s "
            f"lost_timeout={args.lost_timeout:.3f}s",
            flush=True,
        )
        print("Press Ctrl-C to stop.", flush=True)

    def on_low_state(self, msg: LowState) -> None:
        if len(msg.motor_state_serial) >= 2:
            self.head_yaw = float(msg.motor_state_serial[0].q)
            self.head_pitch = float(msg.motor_state_serial[1].q)

    def on_camera_info(self, msg: CameraInfo) -> None:
        if msg.width > 0 and msg.height > 0:
            self.image_width = float(msg.width)
            self.image_height = float(msg.height)

        fx = float(msg.k[0]) if len(msg.k) > 0 else 0.0
        fy = float(msg.k[4]) if len(msg.k) > 4 else 0.0
        if fx > 0.0 and self.image_width > 0.0:
            self.fov_x = 2.0 * math.atan(self.image_width / (2.0 * fx))
        if fy > 0.0 and self.image_height > 0.0:
            self.fov_y = 2.0 * math.atan(self.image_height / (2.0 * fy))

    def on_detections(self, msg: Detections) -> None:
        best: Optional[BallObservation] = None
        for obj in msg.detected_objects:
            if obj.label.strip().lower() != "ball":
                continue
            confidence = float(obj.confidence)
            if confidence < self.args.min_confidence:
                continue
            xmin = float(obj.xmin)
            ymin = float(obj.ymin)
            xmax = float(obj.xmax)
            ymax = float(obj.ymax)
            if xmax <= xmin or ymax <= ymin:
                continue
            if not all(valid_number(v) for v in (xmin, ymin, xmax, ymax)):
                continue

            obs = BallObservation(
                confidence=confidence,
                cx=(xmin + xmax) * 0.5,
                cy=(ymin + ymax) * 0.5,
                width=xmax - xmin,
                height=ymax - ymin,
            )
            if best is None or obs.confidence > best.confidence:
                best = obs

        if best is not None:
            now = time.monotonic()
            if self.latest_ball is not None and self.latest_ball_time > 0.0:
                dt = now - self.latest_ball_time
                if 0.001 <= dt <= self.args.velocity_timeout:
                    raw_vx = (best.cx - self.latest_ball.cx) / dt
                    raw_vy = (best.cy - self.latest_ball.cy) / dt
                    alpha = clamp(self.args.ball_velocity_alpha, 0.0, 1.0)
                    self.filtered_ball_vx = alpha * raw_vx + (1.0 - alpha) * self.filtered_ball_vx
                    self.filtered_ball_vy = alpha * raw_vy + (1.0 - alpha) * self.filtered_ball_vy
                else:
                    self.filtered_ball_vx = 0.0
                    self.filtered_ball_vy = 0.0

            self.latest_ball = best
            self.latest_ball_time = now

    def measured_pitch(self) -> float:
        if self.head_pitch is not None and valid_number(self.head_pitch):
            return self.head_pitch
        return self.command_pitch

    def measured_yaw(self) -> float:
        if self.head_yaw is not None and valid_number(self.head_yaw):
            return self.head_yaw
        return self.command_yaw

    def publish_head_target(self, pitch: float, yaw: float) -> None:
        if not self.args.dry_run:
            msg = RpcReqMsg()
            msg.uuid = str(uuid.uuid4())
            msg.header = json.dumps({"api_id": self.args.absolute_api_id})
            msg.body = json.dumps({"pitch": float(pitch), "yaw": float(yaw)})
            self.publisher.publish(msg)
        self.last_command_time = time.monotonic()

    def reset_pd_state(self) -> None:
        self.prev_yaw_error = None
        self.prev_pitch_error = None
        self.prev_error_time = 0.0
        self.filtered_yaw_error_rate = 0.0
        self.filtered_pitch_error_rate = 0.0

    def predicted_ball(self, now: float) -> tuple[Optional[BallObservation], float, bool]:
        if self.latest_ball is None or self.latest_ball_time <= 0.0:
            return None, float("inf"), False

        age = now - self.latest_ball_time
        if age > self.args.lost_timeout:
            return None, age, False

        horizon = clamp(age + self.args.lead_time, 0.0, self.args.max_prediction)
        cx = self.latest_ball.cx + self.filtered_ball_vx * horizon
        cy = self.latest_ball.cy + self.filtered_ball_vy * horizon
        cx = clamp(cx, 0.0, max(self.image_width - 1.0, 0.0))
        cy = clamp(cy, 0.0, max(self.image_height - 1.0, 0.0))
        predicted = horizon > 0.0 and (
            abs(self.filtered_ball_vx) > self.args.min_prediction_speed
            or abs(self.filtered_ball_vy) > self.args.min_prediction_speed
        )

        return (
            BallObservation(
                confidence=self.latest_ball.confidence,
                cx=cx,
                cy=cy,
                width=self.latest_ball.width,
                height=self.latest_ball.height,
            ),
            age,
            predicted,
        )

    def pd_steps(
        self,
        now: float,
        yaw_error: float,
        pitch_error: float,
    ) -> tuple[float, float, float, float, float, float]:
        control_dt = 1.0 / max(self.args.command_hz, 1.0)
        if self.last_command_time > 0.0:
            control_dt = clamp(now - self.last_command_time, 0.001, 0.25)

        yaw_error_rate = 0.0
        pitch_error_rate = 0.0
        if (
            self.prev_yaw_error is not None
            and self.prev_pitch_error is not None
            and self.prev_error_time > 0.0
        ):
            error_dt = clamp(now - self.prev_error_time, 0.001, 0.25)
            raw_yaw_rate = (yaw_error - self.prev_yaw_error) / error_dt
            raw_pitch_rate = (pitch_error - self.prev_pitch_error) / error_dt
            alpha = clamp(self.args.derivative_alpha, 0.0, 1.0)
            self.filtered_yaw_error_rate = (
                alpha * raw_yaw_rate + (1.0 - alpha) * self.filtered_yaw_error_rate
            )
            self.filtered_pitch_error_rate = (
                alpha * raw_pitch_rate + (1.0 - alpha) * self.filtered_pitch_error_rate
            )
            yaw_error_rate = self.filtered_yaw_error_rate
            pitch_error_rate = self.filtered_pitch_error_rate

        self.prev_yaw_error = yaw_error
        self.prev_pitch_error = pitch_error
        self.prev_error_time = now

        yaw_rate = self.args.yaw_kp * yaw_error + self.args.yaw_kd * yaw_error_rate
        pitch_rate = self.args.pitch_kp * pitch_error + self.args.pitch_kd * pitch_error_rate

        yaw_rate = clamp(yaw_rate, -self.args.max_yaw_rate, self.args.max_yaw_rate)
        pitch_rate = clamp(pitch_rate, -self.args.max_pitch_rate, self.args.max_pitch_rate)

        yaw_step = clamp(yaw_rate * control_dt, -self.args.max_step, self.args.max_step)
        pitch_step = clamp(pitch_rate * control_dt, -self.args.max_step, self.args.max_step)
        return pitch_step, yaw_step, pitch_rate, yaw_rate, pitch_error_rate, yaw_error_rate

    def tick(self) -> None:
        now = time.monotonic()
        ball, age, predicted = self.predicted_ball(now)

        if ball is None:
            if self.was_tracking:
                print(
                    f"[LOST] no fresh ball for {age:.2f}s; holding last target "
                    f"pitch={self.command_pitch:+.3f} yaw={self.command_yaw:+.3f}",
                    flush=True,
                )
            self.was_tracking = False
            self.reset_pd_state()
            return

        self.was_tracking = True

        center_x = self.image_width * 0.5
        center_y = self.image_height * 0.5
        if center_x <= 0.0 or center_y <= 0.0:
            return

        dx = ball.cx - center_x
        dy = ball.cy - center_y

        yaw_error = 0.0
        pitch_error = 0.0
        if abs(dx) > self.args.deadband_px:
            yaw_error = -(dx / center_x) * (self.fov_x * 0.5)
        if abs(dy) > self.args.deadband_px:
            pitch_error = (dy / center_y) * (self.fov_y * 0.5)

        outside_deadband = yaw_error != 0.0 or pitch_error != 0.0
        if not outside_deadband:
            self.reset_pd_state()

        pitch_step, yaw_step, pitch_rate, yaw_rate, pitch_error_rate, yaw_error_rate = self.pd_steps(
            now, yaw_error, pitch_error
        )

        target_yaw = clamp(self.measured_yaw() + yaw_step, self.args.min_yaw, self.args.max_yaw)
        target_pitch = clamp(self.measured_pitch() + pitch_step, self.args.min_pitch, self.args.max_pitch)

        moved_enough = (
            abs(target_yaw - self.command_yaw) >= self.args.min_command_delta
            or abs(target_pitch - self.command_pitch) >= self.args.min_command_delta
        )
        can_publish = now - self.last_command_time >= self.args.min_command_period

        if outside_deadband and moved_enough and can_publish:
            self.command_yaw = target_yaw
            self.command_pitch = target_pitch
            self.publish_head_target(target_pitch, target_yaw)
            self.print_status(
                "CMD",
                ball,
                dx,
                dy,
                pitch_step,
                yaw_step,
                pitch_rate,
                yaw_rate,
                pitch_error_rate,
                yaw_error_rate,
                age,
                predicted,
            )
            return

        if now - self.last_print_time >= self.args.status_period:
            label = "CENTER" if not outside_deadband else "WAIT"
            self.print_status(
                label,
                ball,
                dx,
                dy,
                pitch_step,
                yaw_step,
                pitch_rate,
                yaw_rate,
                pitch_error_rate,
                yaw_error_rate,
                age,
                predicted,
            )

    def print_status(
        self,
        label: str,
        ball: BallObservation,
        dx: float,
        dy: float,
        pitch_step: float,
        yaw_step: float,
        pitch_rate: float,
        yaw_rate: float,
        pitch_error_rate: float,
        yaw_error_rate: float,
        age: float,
        predicted: bool,
    ) -> None:
        self.last_print_time = time.monotonic()
        mode = "predict" if predicted else "detect"
        print(
            f"[{label}] {mode} age={age:.2f}s conf={ball.confidence:.1f} "
            f"center=({ball.cx:.0f},{ball.cy:.0f}) "
            f"err=({dx:+.0f},{dy:+.0f})px "
            f"ball_v=({self.filtered_ball_vx:+.0f},{self.filtered_ball_vy:+.0f})px/s "
            f"rate pitch={pitch_rate:+.3f} yaw={yaw_rate:+.3f} "
            f"step pitch={pitch_step:+.3f} yaw={yaw_step:+.3f} "
            f"d_err pitch={pitch_error_rate:+.3f} yaw={yaw_error_rate:+.3f} "
            f"target pitch={self.command_pitch:+.3f} yaw={self.command_yaw:+.3f} "
            f"measured pitch={self.measured_pitch():+.3f} yaw={self.measured_yaw():+.3f}",
            flush=True,
        )


def run_stop_script() -> None:
    if not os.path.exists("./scripts/stop.sh"):
        print("[WARN] ./scripts/stop.sh not found; skipping stop-first.", flush=True)
        return
    subprocess.run(["./scripts/stop.sh"], check=False)


def start_vision(args: argparse.Namespace) -> Optional[subprocess.Popen]:
    if not args.start_vision:
        return None
    if args.stop_first:
        run_stop_script()
        time.sleep(2.0)

    print("---- start vision only ----", flush=True)
    log_file = open(args.vision_log, "a", encoding="utf-8")
    process = subprocess.Popen(
        ["ros2", "launch", "vision", "launch.py"],
        stdout=log_file,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    print(f"Vision log: {args.vision_log}", flush=True)
    time.sleep(args.vision_wait)
    return process


def stop_started_vision(process: Optional[subprocess.Popen]) -> None:
    if process is None or process.poll() is not None:
        return
    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Direct Python ball-to-head tracker using YOLO detections and RotateHead API 2004."
    )
    parser.add_argument("--start-vision", action="store_true", help="start vision before tracking")
    parser.add_argument("--no-stop-first", dest="stop_first", action="store_false", help="do not run scripts/stop.sh before starting vision")
    parser.add_argument("--stop-vision-on-exit", action="store_true", help="stop the vision process started by this script on exit")
    parser.add_argument("--vision-wait", type=float, default=8.0, help="seconds to wait after starting vision")
    parser.add_argument("--vision-log", default="vision.log", help="vision log path")

    parser.add_argument("--detection-topic", default="/booster_soccer/detection", help="YOLO detections topic")
    parser.add_argument("--low-state-topic", default="/low_state", help="measured head joint topic")
    parser.add_argument("--camera-info-topic", default="/boostercamera/head/rgb/camera_info", help="camera info topic; empty disables it")
    parser.add_argument("--loco-topic", default="/LocoApiTopicReq", help="RPC request topic")
    parser.add_argument("--absolute-api-id", type=int, default=2004, help="RotateHead absolute API id")

    parser.add_argument("--image-width", type=float, default=1280.0, help="fallback image width in pixels")
    parser.add_argument("--image-height", type=float, default=720.0, help="fallback image height in pixels")
    parser.add_argument("--fov-x-deg", type=float, default=90.0, help="fallback horizontal FOV")
    parser.add_argument("--fov-y-deg", type=float, default=65.0, help="fallback vertical FOV")

    parser.add_argument("--min-confidence", type=float, default=30.0, help="minimum Ball confidence")
    parser.add_argument("--deadband-px", type=float, default=15.0, help="pixel error ignored around image center")
    parser.add_argument("--yaw-kp", "--yaw-gain", dest="yaw_kp", type=float, default=1.00, help="horizontal proportional gain")
    parser.add_argument("--pitch-kp", "--pitch-gain", dest="pitch_kp", type=float, default=1.00, help="vertical proportional gain")
    parser.add_argument("--yaw-kd", type=float, default=0.08, help="horizontal derivative gain")
    parser.add_argument("--pitch-kd", type=float, default=0.08, help="vertical derivative gain")
    parser.add_argument("--derivative-alpha", type=float, default=0.35, help="derivative low-pass filter alpha, 0..1")
    parser.add_argument("--max-yaw-rate", type=float, default=0.65, help="maximum yaw command speed in radians/sec")
    parser.add_argument("--max-pitch-rate", type=float, default=0.55, help="maximum pitch command speed in radians/sec")
    parser.add_argument("--max-step", type=float, default=0.055, help="extra safety clamp for pitch/yaw step per command in radians")
    parser.add_argument("--command-hz", type=float, default=12.0, help="control loop rate")
    parser.add_argument("--min-command-period", type=float, default=0.08, help="minimum seconds between head commands")
    parser.add_argument("--min-command-delta", type=float, default=0.004, help="minimum target change before publishing")
    parser.add_argument("--lost-timeout", type=float, default=0.45, help="seconds to keep predicting before ball is considered lost")
    parser.add_argument("--lead-time", type=float, default=0.08, help="seconds to lead the detected ball position")
    parser.add_argument("--max-prediction", type=float, default=0.25, help="maximum seconds of image-plane prediction")
    parser.add_argument("--ball-velocity-alpha", type=float, default=0.45, help="ball image velocity low-pass alpha, 0..1")
    parser.add_argument("--velocity-timeout", type=float, default=0.5, help="reset ball velocity after this gap between detections")
    parser.add_argument("--min-prediction-speed", type=float, default=20.0, help="minimum ball image speed in px/s before status is marked predict")
    parser.add_argument("--status-period", type=float, default=0.5, help="seconds between non-command status prints")

    parser.add_argument("--min-yaw", type=float, default=-1.10, help="right yaw limit")
    parser.add_argument("--max-yaw", type=float, default=1.10, help="left yaw limit")
    parser.add_argument("--min-pitch", type=float, default=-0.314, help="up pitch limit")
    parser.add_argument("--max-pitch", type=float, default=0.75, help="down pitch limit")
    parser.add_argument("--initial-pitch", type=float, default=0.35, help="fallback pitch before low_state arrives")
    parser.add_argument("--initial-yaw", type=float, default=0.0, help="fallback yaw before low_state arrives")
    parser.add_argument("--dry-run", action="store_true", help="print commands without publishing them")
    parser.set_defaults(stop_first=True)
    return parser


def main() -> int:
    args = build_parser().parse_args()

    if args.command_hz <= 0.0:
        print("--command-hz must be positive", file=sys.stderr)
        return 2
    if args.image_width <= 0.0 or args.image_height <= 0.0:
        print("--image-width and --image-height must be positive", file=sys.stderr)
        return 2

    vision_process = start_vision(args)
    rclpy.init()
    node = DirectBallHeadTracker(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopping direct ball head tracker.", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if args.stop_vision_on_exit:
            stop_started_vision(vision_process)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
