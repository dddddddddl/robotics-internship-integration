#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read six signed actual-force register channels through ROS 2 and end-effector RS485 (A/COM2/ID1). Sends only 0x11 register reads. Values retain device units; no force calibration is assumed."""

import argparse
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

from marvin_interfaces.srv import Connect, Transfer485


ROBOT_IP = "192.168.1.190"
ARM_ID = "A"
COM = 2
HAND_ID = 1


FORCE_ACT_ADDRESS = 1582
FORCE_COUNT = 6

DOF_NAMES = (
    "Pinky",
    "Ring",
    "Middle",
    "Index",
    "Thumb flexion",
    "Thumb rotation",
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read-only left-hand six-channel force monitor over RS485"
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=0.5,
        help="Read frequency; default 0.5 Hz (every 2 seconds)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="RS485 response timeout in seconds; default 2.0",
    )
    parser.add_argument(
        "--max-failures",
        type=int,
        default=3,
        help="Stop after this many consecutive failures; default 3",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Read once and exit",
    )
    return parser.parse_args()


def validate_args(args):
    if not 0.1 <= args.hz <= 2.0:
        raise ValueError("hz must be in [0.1, 2.0]")
    if not 0.1 <= args.timeout <= 5.0:
        raise ValueError("timeout must be in [0.1, 5.0] seconds")
    if not 1 <= args.max_failures <= 10:
        raise ValueError("max-failures must be in [1, 10]")


def bytes_to_hex(data):
    return " ".join(f"{value:02X}" for value in data)


def hex_to_bytes(text):
    cleaned = "".join(str(text).split())
    if len(cleaned) % 2:
        raise ValueError("RS485 response has an odd number of hexadecimal characters")
    try:
        return bytes.fromhex(cleaned)
    except ValueError as exc:
        raise ValueError(f"Invalid hexadecimal RS485 response: {text}") from exc


def build_read_request(hand_id, address, count):
    """Build read request."""
    byte_count = count * 2
    frame = bytearray(
        [
            0xEB,
            0x90,
            hand_id,
            0x04,
            0x11,
            address & 0xFF,
            (address >> 8) & 0xFF,
            byte_count,
        ]
    )
    frame.append(sum(frame[2:]) & 0xFF)
    return bytes_to_hex(frame)


def find_frame_start(data):
    for index in range(len(data) - 1):
        if data[index : index + 2] in (b"\xEB\x90", b"\x90\xEB"):
            return index
    raise ValueError("No EB 90/90 EB header in RS485 response")


def unsigned_to_signed(value):
    value &= 0xFFFF
    return value - 0x10000 if value >= 0x8000 else value


def parse_read_response(rx_hex):
    """Parse read response."""
    data = hex_to_bytes(rx_hex)
    frame = data[find_frame_start(data) :]

    if len(frame) < 8:
        raise ValueError(f"RS485 response too short: {rx_hex}")
    if frame[2] != HAND_ID:
        raise ValueError(
            f"Device ID mismatch: expected {HAND_ID}, received {frame[2]}"
        )

    payload_length = frame[3]
    frame_length = payload_length + 5
    if len(frame) < frame_length:
        raise ValueError(f"Incomplete RS485 response: {rx_hex}")
    frame = frame[:frame_length]

    if (sum(frame[2:-1]) & 0xFF) != frame[-1]:
        raise ValueError(f"RS485 checksum mismatch: {rx_hex}")
    if frame[4] != 0x11:
        raise ValueError(f"Unexpected RS485 opcode: 0x{frame[4]:02X}")

    address = frame[5] | (frame[6] << 8)
    if address != FORCE_ACT_ADDRESS:
        raise ValueError(
            f"Register address mismatch: expected {FORCE_ACT_ADDRESS}, received {address}"
        )

    byte_count = payload_length - 3
    expected_bytes = FORCE_COUNT * 2
    if byte_count != expected_bytes:
        raise ValueError(
            f"Payload size mismatch: expected {expected_bytes}, received {byte_count}"
        )

    payload = frame[7 : 7 + byte_count]
    values = [
        payload[index] | (payload[index + 1] << 8)
        for index in range(0, len(payload), 2)
    ]
    return [unsigned_to_signed(value) for value in values]


def format_forces(values):
    return " | ".join(
        f"{name}:{value:5d} device units"
        for name, value in zip(DOF_NAMES, values)
    )


class LeftHandForceReader(Node):
    def __init__(self):
        super().__init__("left_hand_force_485_read_only")
        self.connect_client = self.create_client(
            Connect, "/arm/connect_robot"
        )
        self.transfer_client = self.create_client(
            Transfer485, "/arm/transfer_485"
        )

    @staticmethod
    def wait_for_service(client, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Waiting for service {client.srv_name} timed out")

    def call_service(self, client, request, timeout_sec):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=timeout_sec,
        )

        if not future.done():
            future.cancel()
            raise RuntimeError(f"Service {client.srv_name} response timed out")
        if future.exception() is not None:
            raise RuntimeError(
                f"Service {client.srv_name} exception: {future.exception()}"
            )

        response = future.result()
        if response is None:
            raise RuntimeError(f"Service {client.srv_name} returned no result")
        return response

    def connect_robot(self):
        self.wait_for_service(self.connect_client)
        request = Connect.Request()
        request.robot_ip = ROBOT_IP
        response = self.call_service(
            self.connect_client,
            request,
            timeout_sec=12.0,
        )

        if response.success:
            print(response.message)
            return
        if "Already connected" in response.message:
            print("Robot controller already connected.")
            return
        raise RuntimeError(f"Robot connection failed: {response.message}")

    def read_forces(self, timeout_sec):
        self.wait_for_service(self.transfer_client)
        tx_hex = build_read_request(
            HAND_ID,
            FORCE_ACT_ADDRESS,
            FORCE_COUNT,
        )

        request = Transfer485.Request()
        request.arm_id = ARM_ID
        request.com = COM
        request.tx_hex = tx_hex
        request.timeout_sec = float(timeout_sec)

        response = self.call_service(
            self.transfer_client,
            request,
            timeout_sec=max(4.0, timeout_sec + 2.0),
        )
        if not response.success:
            raise RuntimeError(
                f"{response.message}; "
                f"tx={tx_hex}; "
                f"rx={response.rx_hex or '<empty>'}"
            )

        return parse_read_response(response.rx_hex)


def main():
    args = parse_args()
    validate_args(args)

    rclpy.init(args=[])
    node = LeftHandForceReader()

    print("Inspire left-hand RS485 actual-force monitor (read-only)")
    print(
        f"Connection: ROS 2 -> {ROBOT_IP} -> "
        f"{ARM_ID}/COM{COM}/ID{HAND_ID}"
    )
    print("Read-only: sends 0x11 reads; no register writes or motion commands.")
    print("Lightly press the fingertip to observe changes; do not force the joints.")

    try:
        node.connect_robot()
        period = 1.0 / args.hz
        consecutive_failures = 0

        while rclpy.ok():
            started = time.monotonic()

            try:
                forces = node.read_forces(args.timeout)
                consecutive_failures = 0
                stamp = datetime.now().isoformat(
                    timespec="milliseconds"
                )
                print(f"[{stamp}] {format_forces(forces)}")
            except (RuntimeError, ValueError) as exc:
                consecutive_failures += 1
                print(
                    f"Read failed (consecutive {consecutive_failures}/"
                    f"{args.max_failures}  attempts): {exc}"
                )

                if "Robot is not connected" in str(exc):
                    try:
                        print("Reconnecting to the robot controller...")
                        node.connect_robot()
                    except RuntimeError as reconnect_exc:
                        print(f"Reconnection failed: {reconnect_exc}")

                if args.once or consecutive_failures >= args.max_failures:
                    print("Monitor stopped; no write or motion commands sent.")
                    return 1

            if args.once:
                return 0

            remaining = period - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print("\nCtrl+C received. Read-only monitor stopped.")
        return 0
    except (RuntimeError, ValueError) as exc:
        print(f"Startup failed: {exc}")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
