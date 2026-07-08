#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
import time

SAFE_LIMIT_DEG = 50.0
HARD_LIMIT_DEG = 55.0
DEFAULT_TOPIC = "LocoApiTopicReq"
DEFAULT_BODY_TEMPLATE = '{"yaw": {radians}}'


def parse_args():
    parser = argparse.ArgumentParser(
        description="Safely test waist yaw command payloads through LocoApiTopicReq."
    )
    parser.add_argument(
        "angles_degrees",
        nargs="+",
        type=float,
        help="Target waist yaw angle(s) in degrees. Example: 10 -10 0",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually publish commands. Without this flag the script only prints them.",
    )
    parser.add_argument(
        "--api-id",
        type=int,
        default=None,
        help="Confirmed Booster waist yaw API id. Required with --execute unless WAIST_YAW_API_ID is set.",
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help=f"RPC request topic. Default: {DEFAULT_TOPIC}",
    )
    parser.add_argument(
        "--body-template",
        default=DEFAULT_BODY_TEMPLATE,
        help=(
            "JSON template for the request body. Available placeholders: "
            "{degrees}, {radians}. Default: " + DEFAULT_BODY_TEMPLATE
        ),
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=1.0,
        help="Seconds to wait between published angle commands. Default: 1.0",
    )
    parser.add_argument(
        "--return-zero",
        action="store_true",
        help="Append 0 degrees after the requested sequence.",
    )
    parser.add_argument(
        "--allow-near-limit",
        action="store_true",
        help=f"Allow commands up to {HARD_LIMIT_DEG} degrees instead of the default {SAFE_LIMIT_DEG}.",
    )
    return parser.parse_args()


def validate_angles(angles_degrees, allow_near_limit):
    limit = HARD_LIMIT_DEG if allow_near_limit else SAFE_LIMIT_DEG
    for angle in angles_degrees:
        if abs(angle) > limit:
            raise ValueError(
                f"Refusing {angle} degrees. Limit for this test is +/-{limit} degrees."
            )


def resolve_api_id(api_id):
    if api_id is not None:
        return api_id

    env_value = os.environ.get("WAIST_YAW_API_ID", "").strip()
    if env_value:
        return int(env_value)

    return None


def render_body(template, degrees):
    radians = math.radians(degrees)
    body_text = template.replace("{degrees}", str(degrees)).replace("{radians}", str(radians))
    return json.dumps(json.loads(body_text), separators=(",", ":"))


def build_commands(angles_degrees, body_template, return_zero):
    sequence = list(angles_degrees)
    if return_zero and (not sequence or abs(sequence[-1]) > 1e-9):
        sequence.append(0.0)

    commands = []
    for degrees in sequence:
        commands.append(
            {
                "degrees": degrees,
                "radians": math.radians(degrees),
                "body": render_body(body_template, degrees),
            }
        )
    return commands


def publish_commands(topic, api_id, commands, hold):
    import rclpy
    from booster_msgs.msg import RpcReqMsg

    rclpy.init()
    node = rclpy.create_node("waist_yaw_motion_test")
    publisher = node.create_publisher(RpcReqMsg, topic, 10)
    time.sleep(0.5)

    try:
        for index, command in enumerate(commands, start=1):
            msg = RpcReqMsg()
            msg.uuid = f"waist-yaw-test-{int(time.time() * 1000)}-{index}"
            msg.header = json.dumps({"api_id": api_id}, separators=(",", ":"))
            msg.body = command["body"]

            print(
                f"Publishing {command['degrees']:.1f} deg "
                f"({command['radians']:.4f} rad) to {topic}"
            )
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(max(0.0, hold))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    args = parse_args()

    try:
        validate_angles(args.angles_degrees, args.allow_near_limit)
        commands = build_commands(args.angles_degrees, args.body_template, args.return_zero)
    except (json.JSONDecodeError, KeyError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    api_id = resolve_api_id(args.api_id)

    print("Waist yaw motion test:")
    print(f"  topic: {args.topic}")
    print(f"  execute: {args.execute}")
    print(f"  api_id: {api_id if api_id is not None else 'not set'}")
    print(f"  body_template: {args.body_template}")
    print()

    for command in commands:
        print(
            f"  {command['degrees']:>6.1f} deg  "
            f"{command['radians']:>8.4f} rad  body={command['body']}"
        )

    if not args.execute:
        print()
        print("Dry run only. Nothing was published.")
        print("Use --execute with a confirmed --api-id before moving the robot.")
        return 0

    if api_id is None:
        print()
        print("Refusing to publish: --api-id or WAIST_YAW_API_ID is required with --execute.")
        return 1

    print()
    print("Publishing waist yaw commands. Be ready to stop the robot.")
    publish_commands(args.topic, api_id, commands, args.hold)
    return 0


if __name__ == "__main__":
    sys.exit(main())
