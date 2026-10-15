#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 hand motion example adapted from the vendor hand SDK workflow. Left: A/COM2/ID1; right: B/COM3/ID1. Channels: pinky, ring, middle, index, thumb flexion, thumb rotation. --dry-run prints requests without sending motion."""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from inspire_hand_interface.srv import SetAngles, SetForces, SetSpeeds


HAND_ID = 1
LEFT_ARM = "A"
LEFT_COM = 2
RIGHT_ARM = "B"
RIGHT_COM = 3


COMMAND_GAP_S = 0.05


DEFAULT_SPEEDS = [800, 800, 800, 800, 800, 800]


DEFAULT_FORCES = [4000, 4000, 4000, 4000, 4000, 4000]

# DEFAULT_FORCES = [1000, 1000, 1000, 1000, 1000, 1000]


DEFAULT_ACTIONS = [
    ("Open both hands", [1000, 1000, 1000, 1000, 1000, 1000], [1000, 1000, 1000, 1000, 1000, 1000]),
    ("Open both hands", [1000, 1000, 1000, 1000, 1000, 0], [1000, 1000, 1000, 1000, 1000, 0]),
    ("Close both hands", [10, 10, 10, 10, 1000, 0], [0, 0, 0, 0, 1000, 0]),
]


class InspireHandRos2Demo(Node):
    """Send sequential speed, force-limit and angle requests to the hand services."""

    def __init__(self):
        super().__init__("inspire_hand_ros2_demo")
        self.set_speeds_cli = self.create_client(SetSpeeds, "/hand/set_speeds")
        self.set_forces_cli = self.create_client(SetForces, "/hand/set_forces")
        self.set_angles_cli = self.create_client(SetAngles, "/hand/set_angles")

    def call(self, client, request):
        """Call."""
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for service {client.srv_name} to become available...")

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.exception() is not None:
            raise RuntimeError(f"{client.srv_name} raised: {future.exception()}")

        response = future.result()
        if hasattr(response, "success") and not response.success:
            raise RuntimeError(f"{client.srv_name} failed: {response.message}")
        return response


class HandTarget:
    """Bus address and arm selection for one hand."""

    def __init__(self, name, arm_id, com, hand_id):
        self.name = name
        self.arm_id = arm_id
        self.com = com
        self.hand_id = hand_id


LEFT_HAND = HandTarget("Left hand", LEFT_ARM, LEFT_COM, HAND_ID)
RIGHT_HAND = HandTarget("Right hand", RIGHT_ARM, RIGHT_COM, HAND_ID)


def parse_six_values(text):
    """Parse six values."""
    values = [int(item.strip()) for item in text.replace(",", " ").split() if item.strip()]
    if len(values) != 6:
        raise argparse.ArgumentTypeError("Six integers required, e.g.: '1000 1000 1000 1000 1000 1000'")
    return values


def parse_custom_action(item):
    """Parse custom action."""
    if ";" not in item:
        raise argparse.ArgumentTypeError("Expected action format: 'six left values;six right values'")
    left_text, right_text = item.split(";", 1)
    return ("Custom action", parse_six_values(left_text), parse_six_values(right_text))


def make_speed_request(target, speeds):
    """Make speed request."""
    request = SetSpeeds.Request()
    request.arm_id = target.arm_id
    request.com = target.com
    request.hand_id = target.hand_id
    request.speeds = list(speeds)
    return request


def make_force_request(target, forces):
    """Make force request."""
    request = SetForces.Request()
    request.arm_id = target.arm_id
    request.com = target.com
    request.hand_id = target.hand_id
    request.forces = list(forces)
    return request


def make_angle_request(target, angles):
    """Make angle request."""
    request = SetAngles.Request()
    request.arm_id = target.arm_id
    request.com = target.com
    request.hand_id = target.hand_id
    request.angles = list(angles)
    return request


def selected_targets(hand_side):
    """Selected targets."""
    if hand_side == "left":
        return [LEFT_HAND]
    if hand_side == "right":
        return [RIGHT_HAND]
    return [LEFT_HAND, RIGHT_HAND]


def call_set_speed(node, target, speeds, dry_run=False):
    """Call set speed."""
    request = make_speed_request(target, speeds)
    if dry_run:
        node.get_logger().warning(
            f"Dry run: {target.name} {target.arm_id}/COM{target.com}/ID{target.hand_id} set_speeds={list(speeds)}"
        )
        return None
    response = node.call(node.set_speeds_cli, request)
    node.get_logger().info(f"{target.name} set_speeds ok: {list(speeds)}, tx={response.tx_hex}")
    return response


def call_set_force(node, target, forces, dry_run=False):
    """Call set force."""
    request = make_force_request(target, forces)
    if dry_run:
        node.get_logger().warning(
            f"Dry run: {target.name} {target.arm_id}/COM{target.com}/ID{target.hand_id} set_forces={list(forces)}"
        )
        return None
    response = node.call(node.set_forces_cli, request)
    node.get_logger().info(f"{target.name} set_forces ok: {list(forces)}, tx={response.tx_hex}")
    return response


def call_set_angle(node, target, angles, dry_run=False):
    """Call set angle."""
    request = make_angle_request(target, angles)
    if dry_run:
        node.get_logger().warning(
            f"Dry run: {target.name} {target.arm_id}/COM{target.com}/ID{target.hand_id} set_angles={list(angles)}"
        )
        return None
    response = node.call(node.set_angles_cli, request)
    node.get_logger().info(f"{target.name} set_angles ok: {list(angles)}, tx={response.tx_hex}")
    return response


def send_speeds(node, targets, speeds, dry_run=False):
    """Send speeds."""
    for index, target in enumerate(targets):
        if index > 0:
            time.sleep(COMMAND_GAP_S)
        call_set_speed(node, target, speeds, dry_run=dry_run)


def send_forces(node, targets, forces, dry_run=False):
    """Send forces."""
    for index, target in enumerate(targets):
        if index > 0:
            time.sleep(COMMAND_GAP_S)
        call_set_force(node, target, forces, dry_run=dry_run)


def send_action(node, targets, left_angles, right_angles, dry_run=False):
    """Send action."""
    for index, target in enumerate(targets):
        if index > 0:
            time.sleep(COMMAND_GAP_S)
        angles = left_angles if target is LEFT_HAND else right_angles
        call_set_angle(node, target, angles, dry_run=dry_run)


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser(
        description="Control hand sequences through /hand ROS 2 services, following the vendor SDK example."
    )
    parser.add_argument("--hand", choices=("both", "left", "right"), default="both", help="Select both, left or right hand")
    parser.add_argument("--speeds", type=parse_six_values, default=DEFAULT_SPEEDS, help="Six speed values")
    parser.add_argument("--forces", type=parse_six_values, default=DEFAULT_FORCES, help="Six device force-limit values")
    parser.add_argument("--action", action="append", type=parse_custom_action, help="Custom action, format: 'six left values;six right values'")
    parser.add_argument("--wait", type=float, default=3.0, help="Hold time per action in seconds")
    parser.add_argument("--loop", action="store_true", help="Repeat until Ctrl+C")
    parser.add_argument("--dry-run", action="store_true", help="Print requests without sending commands")
    return parser.parse_known_args()


def main():
    """Main."""
    options, ros_args = parse_args()
    rclpy.init(args=[sys.argv[0], *ros_args])
    node = InspireHandRos2Demo()

    targets = selected_targets(options.hand)
    actions = options.action if options.action else DEFAULT_ACTIONS

    try:
        node.get_logger().info(
            "Hand ROS2 demo start: "
            f"hand={options.hand}, left={LEFT_ARM}/COM{LEFT_COM}/ID{HAND_ID}, right={RIGHT_ARM}/COM{RIGHT_COM}/ID{HAND_ID}"
        )

        
        send_speeds(node, targets, options.speeds, dry_run=options.dry_run)
        time.sleep(0.2)
        send_forces(node, targets, options.forces, dry_run=options.dry_run)
        time.sleep(1.0)

        while True:
            for action_name, left_angles, right_angles in actions:
                node.get_logger().info(f"Execute action: {action_name}")
                send_action(node, targets, left_angles, right_angles, dry_run=options.dry_run)
                time.sleep(options.wait)
            if not options.loop:
                break
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received. Exiting.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
