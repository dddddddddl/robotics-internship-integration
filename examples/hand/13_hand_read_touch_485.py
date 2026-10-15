#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read five fingertip tactile modules through ROS 2 and end-effector RS485 (A/COM2/ID1). Read 25 registers per fingertip and decode normal/tangential float32 fields. Sends only 0x11 register reads."""

import argparse
import math
import struct
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

from marvin_interfaces.srv import Connect, Transfer485


ROBOT_IP = "192.168.1.190"
ARM_ID = "A"
COM = 2
HAND_ID = 1


TOUCH_ADDRESSES = {
    "pinky": 3000,
    "ring": 3058,
    "middle": 3116,
    "index": 3174,
    "thumb": 3232,
}

FINGER_NAMES = {
    "pinky": "Pinky",
    "ring": "Ring",
    "middle": "Middle",
    "index": "Index",
    "thumb": "Thumb",
}

TOUCH_REGISTER_COUNT = 25
NORMAL_FORCE_INDEX = 16
TANGENT_FORCE_INDEX = 20
READ_GAP_SEC = 0.10


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read-only left-hand five-finger tactile monitor over RS485"
    )
    parser.add_argument(
        "--finger",
        choices=("all", *TOUCH_ADDRESSES.keys()),
        default="all",
        help="Read one finger or all five; default all",
    )
    parser.add_argument(
        "--period",
        type=float,
        default=5.0,
        help="Polling period in seconds; default 5.0",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="Per-finger RS485 timeout in seconds; default 2.0",
    )
    parser.add_argument(
        "--max-failures",
        type=int,
        default=3,
        help="Stop after this many fully failed rounds; default 3",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Read one round and exit",
    )
    return parser.parse_args()


def validate_args(args):
    if not 1.0 <= args.period <= 60.0:
        raise ValueError("period must be in [1.0, 60.0] seconds")
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


def build_read_request(address, count):
    byte_count = count * 2
    frame = bytearray(
        [
            0xEB,
            0x90,
            HAND_ID,
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


def parse_register_response(rx_hex, expected_address, expected_count):
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
    if address != expected_address:
        raise ValueError(
            f"Register address mismatch: expected {expected_address}, received {address}"
        )

    byte_count = payload_length - 3
    expected_bytes = expected_count * 2
    if byte_count != expected_bytes:
        raise ValueError(
            f"Payload size mismatch: expected {expected_bytes}, received {byte_count}"
        )

    payload = frame[7 : 7 + byte_count]
    return [
        payload[index] | (payload[index + 1] << 8)
        for index in range(0, len(payload), 2)
    ]


def registers_to_float(registers, index):
    if index < 0 or index + 1 >= len(registers):
        raise ValueError(f"Tactile float field index out of range: {index}")
    raw = struct.pack(
        "<HH",
        int(registers[index]) & 0xFFFF,
        int(registers[index + 1]) & 0xFFFF,
    )
    value = float(struct.unpack("<f", raw)[0])
    if not math.isfinite(value):
        raise ValueError(f"Non-finite tactile value: {value}")
    return value


def parse_touch_values(registers):
    if len(registers) != TOUCH_REGISTER_COUNT:
        raise ValueError(
            f"Expected tactile register count: {TOUCH_REGISTER_COUNT} registers;"
            f"received {len(registers)} items"
        )
    return {
        "normal_force": registers_to_float(
            registers,
            NORMAL_FORCE_INDEX,
        ),
        "tangent_force": registers_to_float(
            registers,
            TANGENT_FORCE_INDEX,
        ),
    }


class LeftHandTouchReader(Node):
    def __init__(self):
        super().__init__("left_hand_touch_485_read_only")
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
            raise RuntimeError(f"Service {client.srv_name} responsetimed out")
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

    def read_finger(self, finger, timeout_sec):
        address = TOUCH_ADDRESSES[finger]
        tx_hex = build_read_request(address, TOUCH_REGISTER_COUNT)
        self.wait_for_service(self.transfer_client)

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

        registers = parse_register_response(
            response.rx_hex,
            expected_address=address,
            expected_count=TOUCH_REGISTER_COUNT,
        )
        return parse_touch_values(registers)


def selected_fingers(name):
    if name == "all":
        return list(TOUCH_ADDRESSES)
    return [name]


def read_round(node, fingers, timeout_sec):
    results = {}
    for index, finger in enumerate(fingers):
        try:
            results[finger] = {
                "success": True,
                **node.read_finger(finger, timeout_sec),
            }
        except (RuntimeError, ValueError) as exc:
            results[finger] = {
                "success": False,
                "message": str(exc),
            }

        if index < len(fingers) - 1:
            time.sleep(READ_GAP_SEC)
    return results


def print_results(results):
    stamp = datetime.now().isoformat(timespec="milliseconds")
    print(f"\n[{stamp}] Left-hand fingertip tactile readings")

    success_count = 0
    for finger, result in results.items():
        name = FINGER_NAMES[finger]
        if result["success"]:
            success_count += 1
            print(
                f"{name}: "
                f"Normal force={result['normal_force']:.6g} | "
                f"Tangential force={result['tangent_force']:.6g}"
            )
        else:
            print(f"{name}: Read failed — {result['message']}")

    return success_count


def main():
    args = parse_args()
    validate_args(args)
    fingers = selected_fingers(args.finger)

    rclpy.init(args=[])
    node = LeftHandTouchReader()

    target = "All five fingers" if args.finger == "all" else FINGER_NAMES[args.finger]
    print("Inspire left-hand RS485 fingertip tactile monitor (read-only)")
    print(
        f"Connection: ROS 2 -> {ROBOT_IP} -> "
        f"{ARM_ID}/COM{COM}/ID{HAND_ID}"
    )
    print(f"Read target: {target}; fields: Normal forceandTangential force。")
    print("Read-only: sends 0x11 reads; no register writes or motion commands.")
    print("Lightly press the tactile surface; do not forcibly bend the finger.")

    try:
        node.connect_robot()
        consecutive_failures = 0

        while rclpy.ok():
            started = time.monotonic()
            results = read_round(node, fingers, args.timeout)
            success_count = print_results(results)

            if success_count:
                consecutive_failures = 0
            else:
                consecutive_failures += 1
                print(
                    f"All reads failed this round (consecutive {consecutive_failures}/"
                    f"{args.max_failures}  rounds)"
                )

                messages = " ".join(
                    result.get("message", "")
                    for result in results.values()
                )
                if "Robot is not connected" in messages:
                    try:
                        print("Reconnecting to the robot controller...")
                        node.connect_robot()
                    except RuntimeError as reconnect_exc:
                        print(f"Reconnection failed: {reconnect_exc}")

                if consecutive_failures >= args.max_failures:
                    print(
                        "Monitor stopped. If timeouts persist, check whether this hand has"
                        "enabled fingertip tactile sensors."
                    )
                    return 1

            if args.once:
                return 0 if success_count else 1

            remaining = args.period - (time.monotonic() - started)
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
