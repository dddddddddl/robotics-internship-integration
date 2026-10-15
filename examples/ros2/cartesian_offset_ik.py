#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Single-target Cartesian offset experiment. Read the body-frame TCP, apply translation (mm) and fixed-body-axis rotation (degrees), send a body-frame target to the arm IK service, which converts it into the vendor IK frame. Computation is the default; --execute explicitly enables joint commands."""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Optional

import numpy as np
import rclpy

from arm_ros2_common import MarvinServiceClient, arm_index, command, finish
from marvin_interfaces.msg import RobotBodyFrames
from marvin_interfaces.srv import ArmClearErr, IK, SetArmState, SetArmVelAcc, SetJointCmdPos


DEFAULT_ARM_ID = "A"
DEFAULT_DX_MM = 10.0
DEFAULT_DY_MM = -0.0
DEFAULT_DZ_MM = -0.0


DEFAULT_DA_DEG = -0.0
DEFAULT_DB_DEG = -0.0
DEFAULT_DC_DEG = 0.0


DEFAULT_IK_OUT_RANGE = 0
DEFAULT_IK_DEG_123 = 0
DEFAULT_IK_DEG_567 = 0
DEFAULT_IK_ZSP_ANGLE_DEG = 1.0

VELOCITY_PERCENT = 10
ACCELERATION_PERCENT = 10
WAIT_AFTER_COMMAND_SEC = 3.0
BODY_FRAMES_TOPIC = "/frame/arm_body_frames"
ARRIVE_TOLERANCE_DEG = 1.0
CHECK_PERIOD_SEC = 0.2
STATE_TIMEOUT_SEC = 5.0
TARGET_TIMEOUT_SEC = 30.0


class IkCartesianOffsetClient(MarvinServiceClient):
    """Service clients and frame subscription for a Cartesian offset experiment."""

    def __init__(self, arm_id: str):
        super().__init__("ik_cartesian_offset_ros2_demo")
        self.arm_id = arm_id
        ik_service = "/arm/body_ik_left_arm" if arm_id == "A" else "/arm/body_ik_right_arm"
        self.official_ik_cli = self.create_client(IK, ik_service)
        self.clear_cli = self.create_client(ArmClearErr, "/arm/clear_arm_error")
        self.state_cli = self.create_client(SetArmState, "/arm/set_arm_state")
        self.vel_cli = self.create_client(SetArmVelAcc, "/arm/set_arm_vel_acc")
        self.position_cli = self.create_client(SetJointCmdPos, "/arm/set_joint_cmd_pos")
        self.latest_body_frames: Optional[RobotBodyFrames] = None
        self.create_subscription(RobotBodyFrames, BODY_FRAMES_TOPIC, self._body_frames_callback, 10)

    def _body_frames_callback(self, message: RobotBodyFrames) -> None:
        self.latest_body_frames = message


def parse_args():
    parser = argparse.ArgumentParser(description="Single-target Cartesian offset IK experiment")
    parser.add_argument("--arm", default=DEFAULT_ARM_ID, choices=["A", "B", "a", "b"], help="Select arm A/B")
    parser.add_argument("--dx", type=float, default=DEFAULT_DX_MM, help="Body X offset in mm")
    parser.add_argument("--dy", type=float, default=DEFAULT_DY_MM, help="Body Y offset in mm")
    parser.add_argument("--dz", type=float, default=DEFAULT_DZ_MM, help="Body Z offset in mm")
    parser.add_argument("--da", type=float, default=DEFAULT_DA_DEG, help="Body X rotation offset in degrees")
    parser.add_argument("--db", type=float, default=DEFAULT_DB_DEG, help="Body Y rotation offset in degrees")
    parser.add_argument("--dc", type=float, default=DEFAULT_DC_DEG, help="Body Z rotation offset in degrees")
    parser.add_argument("--out-range", type=int, default=DEFAULT_IK_OUT_RANGE, help="IK out_range parameter")
    parser.add_argument("--deg-123", type=int, default=DEFAULT_IK_DEG_123, help="IK deg_123 parameter")
    parser.add_argument("--deg-567", type=int, default=DEFAULT_IK_DEG_567, help="IK deg_567 parameter")
    parser.add_argument("--zsp-angle", type=float, default=DEFAULT_IK_ZSP_ANGLE_DEG, help="IK zsp_angle_deg parameter in degrees")
    parser.add_argument("--frame-timeout", type=float, default=5.0, help="Body-frame topic timeout in seconds")
    parser.add_argument("--target-timeout", type=float, default=TARGET_TIMEOUT_SEC, help="Joint target arrival timeout in seconds")
    parser.add_argument("--keep-enabled", action="store_true", help="Keep position mode enabled after completion")
    parser.add_argument("--allow-state-mismatch", action="store_true", help="Send target despite state mismatch (not recommended)")
    parser.add_argument("--dry-run", action="store_true", help="Calculate and print FK/IK without motion")
    parser.add_argument("--execute", action="store_true", help="Enable motion commands; otherwise dry-run")
    return parser.parse_known_args()


def print_matrix(name: str, matrix: np.ndarray) -> None:
    print(f"\n{name}:")
    print(np.array2string(matrix, precision=8, suppress_small=True))


def matrix_to_rpy_deg(rotation: np.ndarray) -> tuple[float, float, float]:
    sy = math.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
    singular = sy < 1e-9
    if not singular:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = 0.0
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def print_pose_rpy(label: str, matrix: np.ndarray) -> None:
    roll_deg, pitch_deg, yaw_deg = matrix_to_rpy_deg(matrix[0:3, 0:3])
    print(
        f"{label} RPY(deg): "
        f"roll={roll_deg:.7f}, pitch={pitch_deg:.7f}, yaw={yaw_deg:.7f}"
    )


def print_pose_summary(label: str, matrix: np.ndarray) -> None:
    print_matrix(label, matrix)
    print(
        f"{label} XYZ(m): "
        f"({matrix[0,3]:.7f}, {matrix[1,3]:.7f}, {matrix[2,3]:.7f})"
    )
    print_pose_rpy(label, matrix)


def joint_limit_margin_deg(joints: list[float], limit_abs: float = 180.0) -> float:
    if len(joints) != 7:
        return float("nan")
    return min(limit_abs - abs(float(value)) for value in joints)


def simple_joint_limit_report(joints: list[float], label: str, limit_abs: float = 180.0) -> None:
    if len(joints) != 7:
        print(f"{label}: joint count is not 7; cannot check +/-{limit_abs:g} deg simple limits")
        return
    exceeded = []
    for index, value in enumerate(joints, start=1):
        if abs(float(value)) > limit_abs:
            exceeded.append(f"J{index}={float(value):.6f}")
    margin = joint_limit_margin_deg(joints, limit_abs)
    print(f"{label}: +/-{limit_abs:g} deg simple limit margin = {margin:.6g} deg")
    if exceeded:
        print(f"{label}: simple check exceeded joints: {', '.join(exceeded)}")


def print_ik_failure_diagnosis(message: str, ik_joints: list[float], ref_joints: list[float]) -> None:
    print("\nIK failure diagnosis:")
    print(f"IK service message: {message if message else 'empty'}")
    simple_joint_limit_report(ref_joints, "IK reference joints")
    if ik_joints:
        simple_joint_limit_report(ik_joints, "IK returned joints")
    else:
        print("IK returned joints: empty; service did not provide joint values to inspect")

    lower_message = message.lower() if message else ""
    if "tags=" in message:
        tags = message.split("tags=", 1)[1].strip()
        print(f"IK_NSP joint-limit tags: {tags}")
        print("When tags has 7 booleans, True maps to exceeded J1-J7.")
    elif "out of workspace" in lower_message:
        print("Likely reason: target pose is outside reachable workspace. Reduce dx/dy/dz or orientation offset.")
    elif "before ik_nsp" in lower_message:
        print("Likely reason: base body IK failed before IK_NSP configuration optimization; no joint tags are available at this stage.")
        print("Suggestion: reduce dx/dy/dz/da/db/dc, or use a reference posture closer to the target.")
    elif "joint limit" in lower_message:
        print("Likely reason: IK solution exceeded joint limits. If the message includes tags, map them to J1-J7.")
    else:
        print("Likely reason: IK service did not expose finer details; inspect current pose, target pose and reference joints.")


def read_current_body_frame(node: IkCartesianOffsetClient, timeout_sec: float) -> tuple[np.ndarray, list[float]]:
    """Read current body frame."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        message = node.latest_body_frames
        if message is None:
            continue
        if not message.success:
            node.get_logger().info(f"{BODY_FRAMES_TOPIC} not ready: {message.message}")
            continue

        if node.arm_id == "A":
            matrix_data = message.body_left_tcp
            joints = message.left_arm_joints_deg
            arm_name = "left"
        else:
            matrix_data = message.body_right_tcp
            joints = message.right_arm_joints_deg
            arm_name = "right"

        if len(matrix_data) != 16:
            raise RuntimeError(f"{BODY_FRAMES_TOPIC} {arm_name} TCP matrix length error: {len(matrix_data)}")
        if len(joints) != 7:
            raise RuntimeError(f"{BODY_FRAMES_TOPIC} {arm_name} joint length error: {len(joints)}")

        node.get_logger().info(f"Read body TCP and joints from {BODY_FRAMES_TOPIC}")
        return np.array(matrix_data, dtype=float).reshape((4, 4)), [float(value) for value in joints]

    raise RuntimeError(f"Timeout waiting for valid {BODY_FRAMES_TOPIC}")


def rot_x(angle_rad: float) -> np.ndarray:
    """Rot x."""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def rot_y(angle_rad: float) -> np.ndarray:
    """Rot y."""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def rot_z(angle_rad: float) -> np.ndarray:
    """Rot z."""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def apply_body_offset(T_body_tcp: np.ndarray, options) -> np.ndarray:
    """Apply body offset."""
    T_body_target = T_body_tcp.copy()
    T_body_target[0:3, 3] += np.array([options.dx, options.dy, options.dz], dtype=float) / 1000.0

    R_delta = (
        rot_z(math.radians(options.dc))
        @ rot_y(math.radians(options.db))
        @ rot_x(math.radians(options.da))
    )
    T_body_target[0:3, 0:3] = R_delta @ T_body_target[0:3, 0:3]
    return T_body_target


def call_body_ik(node: IkCartesianOffsetClient, target_mat: list[list[float]], ref_joints: list[float], options) -> tuple[bool, str, list[float]]:
    """Call body ik."""
    while not node.official_ik_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(f"Waiting for {node.official_ik_cli.srv_name} ...")

    request = IK.Request()
    request.mat4x4 = [float(value) for row in target_mat for value in row]
    request.ref_joints = [float(value) for value in ref_joints]
    request.out_range = int(options.out_range)
    request.deg_123 = int(options.deg_123)
    request.deg_567 = int(options.deg_567)
    request.zsp_angle_deg = float(options.zsp_angle)

    future = node.official_ik_cli.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.exception() is not None:
        return False, f"{node.official_ik_cli.srv_name} raised: {future.exception()}", []
    response = future.result()
    ik_joints = [float(value) for value in response.ik_joints]
    if not response.success:
        return False, response.message, ik_joints
    if len(ik_joints) != 7:
        return False, f"IK response joint length error: {len(ik_joints)}", ik_joints
    return True, response.message, ik_joints


def read_arm_feedback(node: IkCartesianOffsetClient, arm_id: str) -> dict:
    """Read arm feedback."""
    data = node.robot_data()
    index = arm_index(arm_id)
    if len(data.outputs) <= index or len(data.states) <= index or len(data.inputs) <= index:
        raise RuntimeError(f"robot_data does not contain arm {arm_id}")

    state = data.states[index]
    output = data.outputs[index]
    input_data = data.inputs[index]
    return {
        "state": int(state.cur_state),
        "cmd_state": int(state.cmd_state),
        "err_code": int(state.err_code),
        "joints": [float(value) for value in output.fb_joint_pos],
        "joint_cmd_pos": [float(value) for value in input_data.joint_cmd_pos],
    }


def log_arm_feedback(node: IkCartesianOffsetClient, arm_id: str, label: str) -> dict:
    """Log arm feedback."""
    feedback = read_arm_feedback(node, arm_id)
    node.get_logger().info(
        f"{label} Arm {arm_id}: state={feedback['state']}, cmd_state={feedback['cmd_state']}, "
        f"err={feedback['err_code']}, fb={[round(v, 3) for v in feedback['joints']]}, "
        f"cmd={[round(v, 3) for v in feedback['joint_cmd_pos']]}"
    )
    return feedback


def wait_for_arm_state(node: IkCartesianOffsetClient, arm_id: str, expected_state: int, timeout_sec: float) -> None:
    """Wait for arm state."""
    start_time = time.monotonic()
    last = None
    while time.monotonic() - start_time <= timeout_sec:
        feedback = read_arm_feedback(node, arm_id)
        last = feedback
        if feedback["err_code"] != 0:
            raise RuntimeError(
                f"Arm {arm_id} reports error while waiting state {expected_state}: "
                f"state={feedback['state']}, cmd_state={feedback['cmd_state']}, err={feedback['err_code']}"
            )
        if feedback["state"] == expected_state:
            return
        time.sleep(CHECK_PERIOD_SEC)

    raise RuntimeError(
        f"Arm {arm_id} did not enter state {expected_state} within {timeout_sec}s. "
        f"Last feedback: state={last['state'] if last else 'unknown'}, "
        f"cmd_state={last['cmd_state'] if last else 'unknown'}, "
        f"err={last['err_code'] if last else 'unknown'}"
    )


def max_abs_error(actual: list[float], target: list[float]) -> float:
    """Max abs error."""
    return max(abs(float(a) - float(t)) for a, t in zip(actual, target))


def wait_until_arrived(node: IkCartesianOffsetClient, arm_id: str, target_joints: list[float], timeout_sec: float) -> None:
    """Wait until arrived."""
    start_time = time.monotonic()
    last_log_time = -1.0
    while True:
        feedback = read_arm_feedback(node, arm_id)
        if feedback["err_code"] != 0:
            raise RuntimeError(
                f"Arm {arm_id} reports error while moving: "
                f"state={feedback['state']}, cmd_state={feedback['cmd_state']}, err={feedback['err_code']}"
            )
        if feedback["state"] != 1:
            raise RuntimeError(
                f"Arm {arm_id} left position mode while moving: "
                f"state={feedback['state']}, cmd_state={feedback['cmd_state']}, err={feedback['err_code']}"
            )

        error = max_abs_error(feedback["joints"], target_joints)
        if error <= ARRIVE_TOLERANCE_DEG:
            node.get_logger().info(f"Arm {arm_id} arrived: max error={error:.3f} deg")
            return

        elapsed = time.monotonic() - start_time
        if elapsed > timeout_sec:
            raise RuntimeError(
                f"Arm {arm_id} timeout after {timeout_sec}s: max error={error:.3f} deg, "
                f"fb={[round(v, 3) for v in feedback['joints']]}, "
                f"cmd={[round(v, 3) for v in feedback['joint_cmd_pos']]}, "
                f"target={[round(v, 3) for v in target_joints]}"
            )

        if elapsed - last_log_time >= 1.0:
            node.get_logger().info(
                f"Waiting Arm {arm_id}: max error={error:.3f} deg, "
                f"fb={[round(v, 3) for v in feedback['joints']]}"
            )
            last_log_time = elapsed
        time.sleep(CHECK_PERIOD_SEC)


def main():
    options, ros_args = parse_args()
    arm_id = options.arm.upper()
    
    execute_motion = bool(options.execute and not options.dry_run)

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = IkCartesianOffsetClient(arm_id)
    enabled = False

    try:
        
        current_body_tcp, current_joints = read_current_body_frame(node, options.frame_timeout)

        # Use current measured joints as IK reference joints.
        ik_ref_joints = current_joints
        # Build target TCP in the body frame.
        target_body_tcp = apply_body_offset(current_body_tcp, options)
        target_mat = target_body_tcp.tolist()

        print(f"Arm: {'L_arm' if arm_id == 'A' else 'R_arm'}")
        print("Target coordinate frame: robot body frame")
        print(
            "Target offset: "
            f"dx={options.dx}mm, dy={options.dy}mm, dz={options.dz}mm, "
            f"da={options.da}deg, db={options.db}deg, dc={options.dc}deg"
        )
        print(f"Current joints(deg): {[round(v, 6) for v in current_joints]}")
        print(f"IK reference joints(deg): {[round(v, 6) for v in ik_ref_joints]}")
        print(
            "IK params: "
            f"out_range={options.out_range}, deg_123={options.deg_123}, "
            f"deg_567={options.deg_567}, zsp_angle_deg={options.zsp_angle}"
        )
        simple_joint_limit_report(ik_ref_joints, "IK reference joints")
        print_pose_summary("Current T_body_tcp from /frame/arm_body_frames", current_body_tcp)
        print_pose_summary("Target T_body_tcp after offset", target_body_tcp)

        success, message, target_joints = call_body_ik(node, target_mat, ik_ref_joints, options)

        print(f"\nIK success: {success}")
        print(f"IK joints(deg): {[round(v, 6) for v in target_joints]}")
        if len(target_joints) == 7:
            print(f"Minimum joint limit margin(deg): {joint_limit_margin_deg(target_joints):.6g} (simple +/-180 deg check)")
        else:
            print("Minimum joint limit margin(deg): unavailable because IK did not return 7 joints")
        print(f"Solve status: {message if message else ('converged' if success else 'failed')}")

        if not success:
            print_ik_failure_diagnosis(message, target_joints, ik_ref_joints)
            print("\nIK failed; skip motion command.")
            return

        if not execute_motion:
            node.get_logger().warning("Dry run: IK calculated only. Add --execute to send motion command.")
            return

        # Only execution mode connects the controller and sends motion commands.

        node.connect_robot()
        log_arm_feedback(node, arm_id, "Before configure")

        clear_response = command(node, node.clear_cli, ArmClearErr, arm_id=arm_id)
        node.get_logger().info(f"Arm {arm_id} clear response: {clear_response.message}")
        vel_response = command(node, node.vel_cli, SetArmVelAcc, arm_id=arm_id, vel=VELOCITY_PERCENT, acc=ACCELERATION_PERCENT)
        node.get_logger().info(f"Arm {arm_id} vel/acc response: {vel_response.message}")
        state_response = command(node, node.state_cli, SetArmState, arm_id=arm_id, state=1)
        node.get_logger().info(f"Arm {arm_id} set state response: {state_response.message}")
        if not options.allow_state_mismatch:
            wait_for_arm_state(node, arm_id, expected_state=1, timeout_sec=STATE_TIMEOUT_SEC)
            node.get_logger().info(f"Arm {arm_id} is in position mode")
        enabled = True

        
        command_response = command(node, node.position_cli, SetJointCmdPos, arm_id=arm_id, joints=target_joints)
        node.get_logger().info(f"Arm {arm_id} target response: {command_response.message}")
        node.get_logger().info(f"IK target joints sent, wait {WAIT_AFTER_COMMAND_SEC} sec")
        wait_until_arrived(node, arm_id, target_joints, timeout_sec=options.target_timeout)
        time.sleep(WAIT_AFTER_COMMAND_SEC)

    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received, exit.")
    finally:
        if enabled and not options.keep_enabled:
            try:
                command(node, node.state_cli, SetArmState, arm_id=arm_id, state=0)
                node.get_logger().info(f"Arm {arm_id} disabled")
            except Exception as exc:  # pylint: disable=broad-except
                node.get_logger().error(f"Disable failed: {exc}")
        finish(node)


if __name__ == "__main__":
    main()
