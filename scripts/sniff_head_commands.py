#!/usr/bin/env python3
import argparse
import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from booster_interface.msg import LowState
from booster_msgs.msg import RpcReqMsg


def parse_json(text: str) -> object:
    if not text:
        return {}
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return text


def compact(value: object) -> str:
    if isinstance(value, (dict, list)):
        return json.dumps(value, sort_keys=True)
    return str(value)


class HeadCommandSniffer(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("head_command_sniffer")
        self.args = args
        self.start_time = time.monotonic()
        self.head_yaw: Optional[float] = None
        self.head_pitch: Optional[float] = None
        self.api_filter = {
            int(item.strip())
            for item in args.api_ids.split(",")
            if item.strip()
        }

        self.create_subscription(RpcReqMsg, args.topic, self.on_request, 50)
        if args.low_state_topic:
            self.create_subscription(LowState, args.low_state_topic, self.on_low_state, 10)

        print("---- head command sniffer ----", flush=True)
        print(f"topic={args.topic}", flush=True)
        print(f"api_ids={sorted(self.api_filter) if self.api_filter else 'ALL'}", flush=True)
        print(
            "Move the head from the Booster app now. If nothing prints, the app is "
            "not publishing through this ROS topic.",
            flush=True,
        )

    def on_low_state(self, msg: LowState) -> None:
        if len(msg.motor_state_serial) >= 2:
            self.head_yaw = float(msg.motor_state_serial[0].q)
            self.head_pitch = float(msg.motor_state_serial[1].q)

    def measured_head_text(self) -> str:
        if self.head_pitch is None or self.head_yaw is None:
            return "pitch=? yaw=?"
        return f"pitch={self.head_pitch:+.3f} yaw={self.head_yaw:+.3f}"

    def on_request(self, msg: RpcReqMsg) -> None:
        header = parse_json(msg.header)
        body = parse_json(msg.body)

        api_id = None
        if isinstance(header, dict) and "api_id" in header:
            try:
                api_id = int(header["api_id"])
            except (TypeError, ValueError):
                api_id = None

        if self.api_filter and api_id not in self.api_filter:
            return

        elapsed = time.monotonic() - self.start_time
        print(
            f"[{elapsed:8.3f}s] uuid={msg.uuid} api_id={api_id} "
            f"head={self.measured_head_text()}",
            flush=True,
        )
        print(f"  header={compact(header)}", flush=True)
        print(f"  body={compact(body)}", flush=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Print LocoApiTopic head commands, useful for comparing Booster app joystick output."
    )
    parser.add_argument("--topic", default="/LocoApiTopicReq", help="RPC request topic to sniff")
    parser.add_argument("--api-ids", default="2004,2006", help="comma-separated API ids to print; empty means all")
    parser.add_argument("--low-state-topic", default="/low_state", help="topic for measured head yaw/pitch; empty disables it")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    rclpy.init()
    node = HeadCommandSniffer(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopping sniffer.", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
