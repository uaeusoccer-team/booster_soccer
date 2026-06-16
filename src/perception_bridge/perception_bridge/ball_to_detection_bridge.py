#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from vision_interface.msg import Ball, DetectedObject, Detections


class BallToDetectionBridge(Node):
    def __init__(self):
        super().__init__("ball_to_detection_bridge")

        self.declare_parameter("input_topic", "/booster_vision/ball")
        self.declare_parameter("output_topic", "/booster_soccer/detection")
        self.declare_parameter("min_confidence", 0.5)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.min_confidence = float(self.get_parameter("min_confidence").value)

        output_qos = QoSProfile(depth=1)
        self.publisher = self.create_publisher(Detections, self.output_topic, output_qos)
        self.subscription = self.create_subscription(
            Ball,
            self.input_topic,
            self.ball_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Bridging {self.input_topic} (Ball) -> {self.output_topic} (Detections)"
        )

    def ball_callback(self, msg):
        if msg.confidence < self.min_confidence:
            return

        detected_ball = DetectedObject()
        detected_ball.label = "Ball"
        detected_ball.confidence = self._brain_detection_confidence(msg.confidence)
        detected_ball.xmin = 0
        detected_ball.ymin = 0
        detected_ball.xmax = 0
        detected_ball.ymax = 0
        detected_ball.target_uv = []
        detected_ball.received_pos = []
        detected_ball.position = [float(msg.x), float(msg.y), 0.0]
        detected_ball.position_projection = [float(msg.x), float(msg.y), 0.0]
        detected_ball.position_cam = []
        detected_ball.position_confidence = 1

        detections = Detections()
        detections.header = msg.header
        detections.detected_objects = [detected_ball]
        detections.radar_x = []
        detections.radar_y = []
        detections.corner_pos = [0.0] * 10

        self.publisher.publish(detections)

    @staticmethod
    def _brain_detection_confidence(confidence):
        if confidence <= 1.0:
            return float(confidence) * 100.0
        return float(confidence)


def main(args=None):
    rclpy.init(args=args)
    node = BallToDetectionBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
