#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read-only camera-to-ground point transformation. Subscribe to RobotFrames and Detection2DArray; apply the row-major 4x4 ground_camera transform to each camera-frame detection. The upstream detector must provide metric 3D coordinates in result.pose."""

from __future__ import annotations

from typing import Iterable, Optional

import rclpy
from rclpy.node import Node

from marvin_interfaces.msg import RobotFrames
from vision_msgs.msg import Detection2DArray


HEAD_GROUND_FRAME_TOPIC = "/frame/head_ground_frames"
DETECTION_TOPIC = "/object_detection_result"


class CameraToGroundDemo(Node):
    """Transform metric detection positions into the ground frame."""

    def __init__(self) -> None:
        super().__init__("cam2ground_test")

        
        self._ground_camera: Optional[list[float]] = None

        
        self._six_joints_deg: Optional[list[float]] = None

        self.create_subscription(
            RobotFrames,
            HEAD_GROUND_FRAME_TOPIC,
            self._on_head_ground_frames,
            10,
        )
        self.create_subscription(
            Detection2DArray,
            DETECTION_TOPIC,
            self._on_detection_result,
            10,
        )

        self.get_logger().info(
            f"Waiting for {HEAD_GROUND_FRAME_TOPIC} and {DETECTION_TOPIC} ..."
        )

    def _on_head_ground_frames(self, msg: RobotFrames) -> None:
        """On head ground frames."""
        if not msg.success:
            self.get_logger().warn(f"Head ground frames computation failed: {msg.message}")
            return

        if len(msg.ground_camera) != 16:
            self.get_logger().warn(
                f"Ignoring invalid ground_camera length: {len(msg.ground_camera)}"
            )
            return

        self._ground_camera = list(msg.ground_camera)
        self._six_joints_deg = list(msg.leg_joints_deg[:4]) + list(
            msg.head_joints_deg[:2]
        )
        self.get_logger().debug(
            "Updated T_ground_camera: "
            + ", ".join(f"{value:.6f}" for value in self._ground_camera)
        )

    def _on_detection_result(self, msg: Detection2DArray) -> None:
        """On detection result."""
        if self._ground_camera is None:
            self.get_logger().warn(
                f"No ground_camera matrix; waiting for {HEAD_GROUND_FRAME_TOPIC}"
            )
            return

        if self._six_joints_deg is None:
            self.get_logger().warn(
                f"No leg/head joint data; waiting for {HEAD_GROUND_FRAME_TOPIC}"
            )
            return

        if not msg.detections:
            self.get_logger().info(f"Current {DETECTION_TOPIC} has no detections\n")
            return

        for detection_index, detection in enumerate(msg.detections):
            if not detection.results:
                self.get_logger().info(
                    "Detection[%d] has no classification results\n" % detection_index
                )
                continue

            for result_index, result in enumerate(detection.results):
                position = result.pose.pose.position
                x_ground, y_ground, z_ground = transform_point(
                    self._ground_camera,
                    (position.x, position.y, position.z),
                )

                class_id = result.hypothesis.class_id
                score = result.hypothesis.score
                joint_text = ", ".join(
                    f"{value:.6f}" for value in self._six_joints_deg
                )

                self.get_logger().info(
                    "\n"
                    "Detection[%d] result[%d]  class: %s  confidence: %.3f\n"
                    "Camera position: x=%.6f, y=%.6f, z=%.6f\n"
                    "Ground position: x=%.6f, y=%.6f, z=%.6f\n"
                    "Leg and head joint angles(deg): [%s]\n"
                    "Transform: T_ground_camera from %s\n"
                    % (
                        detection_index,
                        result_index,
                        class_id,
                        score,
                        position.x,
                        position.y,
                        position.z,
                        x_ground,
                        y_ground,
                        z_ground,
                        joint_text,
                        HEAD_GROUND_FRAME_TOPIC,
                    )
                )


def transform_point(
    transform_row_major: Iterable[float],
    point_xyz: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Transform point."""
    t = list(transform_row_major)
    if len(t) != 16:
        raise ValueError(f"Expected a 16-value transform, got {len(t)} values")

    x, y, z = point_xyz
    x_out = t[0] * x + t[1] * y + t[2] * z + t[3]
    y_out = t[4] * x + t[5] * y + t[6] * z + t[7]
    z_out = t[8] * x + t[9] * y + t[10] * z + t[11]
    w_out = t[12] * x + t[13] * y + t[14] * z + t[15]

    if abs(w_out) > 1e-9 and abs(w_out - 1.0) > 1e-9:
        return x_out / w_out, y_out / w_out, z_out / w_out

    return x_out, y_out, z_out


def main() -> None:
    rclpy.init()
    node = CameraToGroundDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
