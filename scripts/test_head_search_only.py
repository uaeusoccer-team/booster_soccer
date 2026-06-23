#!/usr/bin/env python3
import argparse
import json
import os
import signal
import subprocess
import sys
import time
import uuid
from typing import Iterable, Optional

import rclpy
from rclpy.node import Node

from booster_msgs.msg import RpcReqMsg
from booster_interface.msg import LowState
from vision_interface.msg import Detections


def parse_float_list(text: str) -> list[float]:
    return [float(item.strip()) for item in text.split(",") if item.strip()]


def fmt_vec(values: Iterable[float]) -> str:
    vals = list(values)
    if not vals:
        return "[]"
    return "[" + ", ".join(f"{value:.3f}" for value in vals) + "]"


def lerp_points(start: float, end: float, count: int) -> list[float]:
    if count <= 1:
        return [end]
    return [start + (end - start) * index / (count - 1) for index in range(count)]


class HeadSearchOnly(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("head_search_only_test")
        self.args = args
        self.publisher = self.create_publisher(RpcReqMsg, args.loco_topic, 10)
        self.create_subscription(Detections, args.detection_topic, self.on_detections, 10)
        self.create_subscription(LowState, args.low_state_topic, self.on_low_state, 10)

        self.latest_detections: Optional[Detections] = None
        self.latest_detection_time = 0.0
        self.last_detection_print_time = 0.0
        self.last_no_detection_print_time = 0.0
        self.head_yaw: Optional[float] = None
        self.head_pitch: Optional[float] = None
        self.last_command_time = 0.0
        self.last_head_print_time = 0.0

        self.label_filter = {
            label.strip().lower()
            for label in args.labels.split(",")
            if label.strip()
        }

    def on_low_state(self, msg: LowState) -> None:
        if len(msg.motor_state_serial) >= 2:
            self.head_yaw = float(msg.motor_state_serial[0].q)
            self.head_pitch = float(msg.motor_state_serial[1].q)

    def on_detections(self, msg: Detections) -> None:
        self.latest_detections = msg
        self.latest_detection_time = time.monotonic()

    def publish_head_target(self, pitch: float, yaw: float) -> None:
        msg = RpcReqMsg()
        msg.uuid = str(uuid.uuid4())

        if self.args.head_api == "absolute":
            msg.header = json.dumps({"api_id": self.args.absolute_api_id})
            msg.body = json.dumps({"pitch": pitch, "yaw": yaw})
        elif self.args.head_api == "direction":
            pitch_direction, yaw_direction = self.direction_for_target(pitch, yaw)
            msg.header = json.dumps(
                {"api_id": self.args.direction_api_id, "expect_response": True}
            )
            msg.body = json.dumps(
                {
                    "pitch_direction": pitch_direction,
                    "yaw_direction": yaw_direction,
                }
            )
        else:
            return

        self.publisher.publish(msg)
        self.last_command_time = time.monotonic()
        measured = self.measured_head_text()
        print(
            f"[HEAD] command pitch={pitch:+.3f} yaw={yaw:+.3f}"
            f" api={self.args.head_api} measured={measured}",
            flush=True,
        )

    def direction_for_target(self, pitch: float, yaw: float) -> tuple[int, int]:
        if self.head_pitch is None or self.head_yaw is None:
            return 0, 0

        yaw_error = yaw - self.head_yaw
        pitch_error = pitch - self.head_pitch

        pitch_direction = 0
        yaw_direction = 0

        if abs(yaw_error) > self.args.direction_deadband:
            pitch_direction = 1 if yaw_error > 0.0 else -1

        if abs(pitch_error) > self.args.direction_deadband:
            yaw_direction = 1 if pitch_error > 0.0 else -1

        return pitch_direction, yaw_direction

    def publish_direction(
        self, pan_direction: int, tilt_direction: int, reason: str = ""
    ) -> None:
        if self.args.invert_pan:
            pan_direction = -pan_direction
        if self.args.invert_tilt:
            tilt_direction = -tilt_direction

        msg = RpcReqMsg()
        msg.uuid = str(uuid.uuid4())
        msg.header = json.dumps(
            {"api_id": self.args.direction_api_id, "expect_response": True}
        )
        msg.body = json.dumps(
            {
                "pitch_direction": int(pan_direction),
                "yaw_direction": int(tilt_direction),
            }
        )
        self.publisher.publish(msg)
        self.last_command_time = time.monotonic()

        now = time.monotonic()
        should_print = (
            reason
            and now - self.last_head_print_time >= self.args.head_print_period
        )
        if should_print:
            print(
                f"[HEAD_PWM] pan_dir={pan_direction:+d} tilt_dir={tilt_direction:+d}"
                f" {reason} measured={self.measured_head_text()}",
                flush=True,
            )
            self.last_head_print_time = now

    def spin_with_detections(self, duration: float) -> None:
        end_time = time.monotonic() + max(duration, 0.0)
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.maybe_print_detections()

    def pwm_direction(
        self,
        pan_direction: int,
        tilt_direction: int,
        duration: float,
        reason: str = "",
    ) -> None:
        end_time = time.monotonic() + max(duration, 0.0)
        on_time = max(0.02, self.args.pwm_period * self.args.pwm_duty)
        off_time = max(0.02, self.args.pwm_period - on_time)

        while rclpy.ok() and time.monotonic() < end_time:
            self.publish_direction(pan_direction, tilt_direction, reason)
            self.spin_with_detections(min(on_time, end_time - time.monotonic()))
            self.publish_direction(0, 0)
            self.spin_with_detections(min(off_time, end_time - time.monotonic()))

    def move_pitch_to_target(self, target_pitch: float) -> None:
        print(
            f"[PITCH] moving toward {target_pitch:+.3f} with PWM duty "
            f"{self.args.pwm_duty:.2f}",
            flush=True,
        )
        deadline = time.monotonic() + self.args.pitch_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.head_pitch is not None:
                error = target_pitch - self.head_pitch
                if abs(error) <= self.args.pitch_tolerance:
                    break
                tilt_direction = 1 if error > 0.0 else -1
            else:
                tilt_direction = 1 if target_pitch >= 0.0 else -1

            self.pwm_direction(
                0,
                tilt_direction,
                self.args.pwm_period,
                reason=f"pitch_target={target_pitch:+.3f}",
            )

        self.publish_direction(0, 0)
        print(f"[PITCH] done, measured={self.measured_head_text()}", flush=True)

    def sweep_yaw_to_target(self, target_yaw: float, fallback_duration: float) -> None:
        print(
            f"[YAW] sweeping toward {target_yaw:+.3f} with PWM duty "
            f"{self.args.pwm_duty:.2f}",
            flush=True,
        )
        deadline = time.monotonic() + max(
            fallback_duration, self.args.sweep_sec, self.args.yaw_timeout
        )
        open_loop_deadline = time.monotonic() + fallback_duration

        while rclpy.ok():
            if self.head_yaw is not None:
                error = target_yaw - self.head_yaw
                if abs(error) <= self.args.yaw_tolerance:
                    break
                pan_direction = 1 if error > 0.0 else -1
                if time.monotonic() > deadline:
                    print("[YAW] timeout before target; stopping sweep", flush=True)
                    break
            else:
                pan_direction = 1 if target_yaw >= 0.0 else -1
                if time.monotonic() > open_loop_deadline:
                    break

            self.pwm_direction(
                pan_direction,
                0,
                self.args.pwm_period,
                reason=f"yaw_target={target_yaw:+.3f}",
            )

        self.publish_direction(0, 0)
        print(f"[YAW] done, measured={self.measured_head_text()}", flush=True)

    def measured_head_text(self) -> str:
        if self.head_pitch is None or self.head_yaw is None:
            return "pitch=? yaw=?"
        return f"pitch={self.head_pitch:+.3f} yaw={self.head_yaw:+.3f}"

    def matching_objects(self) -> list[object]:
        if self.latest_detections is None:
            return []

        objects = []
        for obj in self.latest_detections.detected_objects:
            if obj.confidence < self.args.min_confidence:
                continue
            if self.label_filter and obj.label.lower() not in self.label_filter:
                continue
            objects.append(obj)
        return objects

    def maybe_print_detections(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self.last_detection_print_time < self.args.print_period:
            return

        objects = self.matching_objects()
        if not objects:
            if now - self.last_no_detection_print_time >= self.args.no_detection_period:
                print(
                    f"[DETECTION] no matching objects yet, head {self.measured_head_text()}",
                    flush=True,
                )
                self.last_no_detection_print_time = now
            return

        age = now - self.latest_detection_time if self.latest_detection_time else 0.0
        print(
            f"[DETECTION] {len(objects)} object(s), age={age:.2f}s,"
            f" head {self.measured_head_text()}",
            flush=True,
        )
        for obj in objects:
            print(
                "  "
                f"label={obj.label} confidence={obj.confidence:.2f} "
                f"bbox=({obj.xmin},{obj.ymin})-({obj.xmax},{obj.ymax}) "
                f"target_uv={fmt_vec(obj.target_uv)} "
                f"position_projection={fmt_vec(obj.position_projection)} "
                f"position={fmt_vec(obj.position)} "
                f"position_cam={fmt_vec(obj.position_cam)} "
                f"received_pos={fmt_vec(obj.received_pos)} "
                f"position_confidence={obj.position_confidence}",
                flush=True,
            )

        self.last_detection_print_time = now

    def hold_target(self, pitch: float, yaw: float, duration: float) -> None:
        self.publish_head_target(pitch, yaw)
        end_time = time.monotonic() + duration

        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.args.head_api == "direction"
                and time.monotonic() - self.last_command_time >= self.args.command_period
            ):
                self.publish_head_target(pitch, yaw)
            self.maybe_print_detections()

    def run_scan(self) -> None:
        pitches = parse_float_list(self.args.pitches)
        if not pitches:
            raise RuntimeError("No pitch levels were provided.")

        print("---- head search only test ----", flush=True)
        print("No brain launch, no tracking, no body motion, no kick.", flush=True)
        print(
            f"pitches={pitches} left_yaw={self.args.left_yaw:+.3f} "
            f"right_yaw={self.args.right_yaw:+.3f} cycles={self.args.cycles} "
            f"sweep_sec={self.args.sweep_sec} waypoints={self.args.waypoints}",
            flush=True,
        )
        print(
            f"detection_topic={self.args.detection_topic} labels="
            f"{self.args.labels or 'ALL'} min_confidence={self.args.min_confidence}",
            flush=True,
        )

        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.args.head_api == "pwm":
            self.run_pwm_scan(pitches)
            return

        for cycle in range(self.args.cycles):
            print(f"---- scan cycle {cycle + 1}/{self.args.cycles} ----", flush=True)
            for pitch_index, pitch in enumerate(pitches):
                left_to_right = pitch_index % 2 == 0
                start_yaw = self.args.left_yaw if left_to_right else self.args.right_yaw
                end_yaw = self.args.right_yaw if left_to_right else self.args.left_yaw
                direction = "left -> right" if left_to_right else "right -> left"
                print(
                    f"[SCAN] pitch level {pitch_index + 1}/{len(pitches)} "
                    f"pitch={pitch:+.3f}, yaw {direction}",
                    flush=True,
                )

                yaw_points = lerp_points(start_yaw, end_yaw, self.args.waypoints)
                self.hold_target(pitch, yaw_points[0], self.args.settle_sec)

                segment_duration = self.args.sweep_sec / max(len(yaw_points) - 1, 1)
                for yaw in yaw_points[1:]:
                    self.hold_target(pitch, yaw, segment_duration)

        self.maybe_print_detections(force=True)
        if self.args.center_on_exit:
            self.hold_target(self.args.center_pitch, 0.0, self.args.settle_sec)

    def run_pwm_scan(self, pitches: list[float]) -> None:
        print(
            "PWM mode uses API 2006 direction pulses, not absolute head targets.",
            flush=True,
        )
        print(
            "API fields follow the T1 app-style mapping seen in old robot code: "
            "pitch_direction pans left/right, yaw_direction tilts up/down.",
            flush=True,
        )

        for cycle in range(self.args.cycles):
            print(f"---- PWM scan cycle {cycle + 1}/{self.args.cycles} ----", flush=True)
            for pitch_index, pitch in enumerate(pitches):
                self.move_pitch_to_target(pitch)

                left_to_right = pitch_index % 2 == 0
                start_yaw = self.args.left_yaw if left_to_right else self.args.right_yaw
                end_yaw = self.args.right_yaw if left_to_right else self.args.left_yaw
                direction = "left -> right" if left_to_right else "right -> left"
                print(
                    f"[SCAN_PWM] pitch level {pitch_index + 1}/{len(pitches)} "
                    f"pitch={pitch:+.3f}, yaw {direction}",
                    flush=True,
                )

                if self.args.seek_sweep_start:
                    self.sweep_yaw_to_target(start_yaw, self.args.sweep_sec)
                self.sweep_yaw_to_target(end_yaw, self.args.sweep_sec)

        self.publish_direction(0, 0)
        self.maybe_print_detections(force=True)
        if self.args.center_on_exit:
            self.move_pitch_to_target(self.args.center_pitch)
            self.sweep_yaw_to_target(0.0, self.args.sweep_sec)


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
        description="Sweep the head search pattern only and print YOLO detections."
    )
    parser.add_argument("--start-vision", action="store_true", help="start vision only before scanning")
    parser.add_argument("--no-stop-first", dest="stop_first", action="store_false", help="do not run scripts/stop.sh before starting vision")
    parser.add_argument("--stop-vision-on-exit", action="store_true", help="stop the vision process started by this script on exit")
    parser.add_argument("--vision-wait", type=float, default=8.0, help="seconds to wait after starting vision")
    parser.add_argument("--vision-log", default="vision.log", help="vision log path")

    parser.add_argument("--pitches", default="0.80,0.65,0.50", help="comma-separated pitch levels in radians")
    parser.add_argument("--left-yaw", type=float, default=1.10, help="left yaw limit in radians")
    parser.add_argument("--right-yaw", type=float, default=-1.10, help="right yaw limit in radians")
    parser.add_argument("--cycles", type=int, default=2, help="number of full 3-pitch scan cycles")
    parser.add_argument("--sweep-sec", type=float, default=7.0, help="seconds for each left/right sweep")
    parser.add_argument("--settle-sec", type=float, default=0.8, help="seconds to settle at the sweep start")
    parser.add_argument("--waypoints", type=int, default=5, help="coarse yaw waypoints per sweep; use 2 for endpoint-only")
    parser.add_argument("--center-on-exit", action="store_true", help="return head to yaw 0 at the end")
    parser.add_argument("--center-pitch", type=float, default=0.65, help="pitch used with --center-on-exit")
    parser.add_argument("--pwm-duty", type=float, default=0.35, help="PWM duty cycle for direction mode, 0.0-1.0")
    parser.add_argument("--pwm-period", type=float, default=0.24, help="PWM pulse period in seconds")
    parser.add_argument("--yaw-tolerance", type=float, default=0.04, help="measured yaw tolerance for PWM sweep")
    parser.add_argument("--pitch-tolerance", type=float, default=0.04, help="measured pitch tolerance for PWM pitch levels")
    parser.add_argument("--pitch-timeout", type=float, default=5.0, help="max seconds to reach each pitch level")
    parser.add_argument("--yaw-timeout", type=float, default=12.0, help="max seconds to reach a measured yaw target")
    parser.add_argument("--head-print-period", type=float, default=0.8, help="seconds between PWM head command logs")
    parser.add_argument("--no-seek-sweep-start", dest="seek_sweep_start", action="store_false", help="do not first move to the starting yaw side")
    parser.add_argument("--invert-pan", action="store_true", help="invert pitch_direction sign for horizontal pan")
    parser.add_argument("--invert-tilt", action="store_true", help="invert yaw_direction sign for vertical tilt")

    parser.add_argument("--labels", default="", help="comma-separated labels to print; default prints all")
    parser.add_argument("--min-confidence", type=float, default=0.0, help="minimum confidence percentage to print")
    parser.add_argument("--print-period", type=float, default=0.5, help="minimum seconds between detection prints")
    parser.add_argument("--no-detection-period", type=float, default=2.0, help="seconds between no-detection messages")

    parser.add_argument("--loco-topic", default="/LocoApiTopicReq", help="head command topic")
    parser.add_argument("--detection-topic", default="/booster_soccer/detection", help="detection topic")
    parser.add_argument("--low-state-topic", default="/low_state", help="low-state topic for measured head angles")
    parser.add_argument("--head-api", choices=["pwm", "absolute", "direction"], default="pwm", help="head command API mode")
    parser.add_argument("--absolute-api-id", type=int, default=2004, help="absolute RotateHead API id")
    parser.add_argument("--direction-api-id", type=int, default=2006, help="direction RotateHead API id")
    parser.add_argument("--direction-deadband", type=float, default=0.02, help="direction mode deadband in radians")
    parser.add_argument("--command-period", type=float, default=0.35, help="direction mode republish period")
    parser.set_defaults(stop_first=True)
    parser.set_defaults(seek_sweep_start=True)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.waypoints < 2:
        print("--waypoints must be at least 2", file=sys.stderr)
        return 2
    if args.pwm_duty <= 0.0 or args.pwm_duty > 1.0:
        print("--pwm-duty must be > 0.0 and <= 1.0", file=sys.stderr)
        return 2
    if args.pwm_period <= 0.0:
        print("--pwm-period must be > 0.0", file=sys.stderr)
        return 2

    vision_process = start_vision(args)

    rclpy.init()
    node = HeadSearchOnly(args)
    try:
        node.run_scan()
    except KeyboardInterrupt:
        print("\nStopping head search test.", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if args.stop_vision_on_exit:
            stop_started_vision(vision_process)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
