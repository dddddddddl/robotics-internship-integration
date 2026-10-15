#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read-only AprilTag pose transformation. Subscribe to /frame/head_ground_frames and /apriltags/poses. Compose T_ground_tag = T_ground_camera @ T_camera_tag. The tag detector and frame publisher must already be running."""

from __future__ import annotations

import math
import time
from typing import Iterable, Optional

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from marvin_interfaces.msg import RobotFrames


HEAD_GROUND_FRAME_TOPIC = "/frame/head_ground_frames"
APRILTAG_POSES_TOPIC = "/apriltags/poses"
PRINT_INTERVAL_SEC = 0.5
WAIT_WARN_INTERVAL_SEC = 1.0


class CameraToGroundRpyDemo(Node):
    """Transform tag positions and orientations into the ground frame."""

    def __init__(self) -> None:
        super().__init__("cam2ground_rpy_test")

        
        self._ground_camera: Optional[list[float]] = None

        
        self._six_joints_deg: Optional[list[float]] = None
        self._last_print_time = 0.0
        self._last_wait_warn_time = 0.0

        self.create_subscription(
            RobotFrames,
            HEAD_GROUND_FRAME_TOPIC,
            self._on_head_ground_frames,
            10,
        )
        self.create_subscription(
            PoseArray,
            APRILTAG_POSES_TOPIC,
            self._on_apriltag_poses,
            10,
        )

        self.get_logger().info(
            f"Waiting for {HEAD_GROUND_FRAME_TOPIC} and {APRILTAG_POSES_TOPIC} ..."
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

    def _on_apriltag_poses(self, msg: PoseArray) -> None:
        """On apriltag poses."""
        if self._ground_camera is None:
            self._warn_waiting(
                f"No ground_camera matrix; waiting for {HEAD_GROUND_FRAME_TOPIC}"
            )
            return

        if self._six_joints_deg is None:
            self._warn_waiting(
                f"No leg/head joint data; waiting for {HEAD_GROUND_FRAME_TOPIC}"
            )
            return

        now = time.monotonic()
        if now - self._last_print_time < PRINT_INTERVAL_SEC:
            return
        self._last_print_time = now

        if not msg.poses:
            self.get_logger().info(f"Current {APRILTAG_POSES_TOPIC} has no tag poses\n")
            return

        for pose_index, pose in enumerate(msg.poses):
            camera_position = pose.position
            camera_quat = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
            camera_rpy_rad = quaternion_to_rpy(camera_quat)
            camera_rpy_deg = tuple(math.degrees(value) for value in camera_rpy_rad)

            ground_position, ground_rotation = transform_pose(
                self._ground_camera,
                (
                    camera_position.x,
                    camera_position.y,
                    camera_position.z,
                ),
                camera_quat,
            )
            ground_rpy_deg = tuple(
                math.degrees(value) for value in rotation_matrix_to_rpy(ground_rotation)
            )

            joint_text = ", ".join(f"{value:.6f}" for value in self._six_joints_deg)

            self.get_logger().info(
                "\n"
                "Tag pose[%d]  header.frame_id: %s\n"
                "Camera position: x=%.6f, y=%.6f, z=%.6f\n"
                "Camera RPY(deg): roll=%.6f, pitch=%.6f, yaw=%.6f\n"
                "Ground position: x=%.6f, y=%.6f, z=%.6f\n"
                "Ground RPY(deg): roll=%.6f, pitch=%.6f, yaw=%.6f\n"
                "Leg and head joint angles(deg): [%s]\n"
                % (
                    pose_index,
                    msg.header.frame_id,
                    camera_position.x,
                    camera_position.y,
                    camera_position.z,
                    camera_rpy_deg[0],
                    camera_rpy_deg[1],
                    camera_rpy_deg[2],
                    ground_position[0],
                    ground_position[1],
                    ground_position[2],
                    ground_rpy_deg[0],
                    ground_rpy_deg[1],
                    ground_rpy_deg[2],
                    joint_text,
                )
            )

    def _warn_waiting(self, text: str) -> None:
        now = time.monotonic()
        if now - self._last_wait_warn_time < WAIT_WARN_INTERVAL_SEC:
            return
        self._last_wait_warn_time = now
        self.get_logger().warn(text)


def transform_pose(
    transform_row_major: Iterable[float],
    point_xyz: tuple[float, float, float],
    quaternion_xyzw: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], list[list[float]]]:
    """Transform pose."""
    t = list(transform_row_major)
    if len(t) != 16:
        raise ValueError(f"Expected a 16-value transform, got {len(t)} values")

    ground_position = transform_point(t, point_xyz)
    ground_camera_rotation = [
        [t[0], t[1], t[2]],
        [t[4], t[5], t[6]],
        [t[8], t[9], t[10]],
    ]
    camera_tag_rotation = quaternion_to_rotation_matrix(quaternion_xyzw)
    ground_tag_rotation = matrix_multiply_3x3(
        ground_camera_rotation,
        camera_tag_rotation,
    )
    return ground_position, ground_tag_rotation


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


def quaternion_to_rotation_matrix(
    quaternion_xyzw: tuple[float, float, float, float],
) -> list[list[float]]:
    """Quaternion to rotation matrix."""
    x, y, z, w = quaternion_xyzw
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("Quaternion norm is zero")

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def quaternion_to_rpy(
    quaternion_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """Quaternion to rpy."""
    return rotation_matrix_to_rpy(quaternion_to_rotation_matrix(quaternion_xyzw))


def rotation_matrix_to_rpy(
    rotation: list[list[float]],
) -> tuple[float, float, float]:
    """Rotation matrix to rpy."""
    r20 = clamp(rotation[2][0], -1.0, 1.0)
    pitch = math.asin(-r20)
    cos_pitch = math.cos(pitch)

    if abs(cos_pitch) > 1e-9:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        roll = 0.0
        yaw = math.atan2(-rotation[0][1], rotation[1][1])

    return roll, pitch, yaw


def matrix_multiply_3x3(
    left: list[list[float]],
    right: list[list[float]],
) -> list[list[float]]:
    """Matrix multiply 3x3."""
    return [
        [
            sum(left[row][index] * right[index][col] for index in range(3))
            for col in range(3)
        ]
        for row in range(3)
    ]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def main() -> None:
    rclpy.init()
    node = CameraToGroundRpyDemo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
