#!/usr/bin/env python3
"""Use ROS 2 services to enable joint drag mode and record arm B data."""

import csv
import os
import signal
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

from marvin_interfaces.srv import (
    ArmClearErr,
    Connect,
    LogSwitch,
    RobotData,
    SetArmState,
    SetDragSpace,
    SetImpedanceType,
    SetJointKD,
)


ARM_ID = "B"
ARM_INDEX = 1
ROBOT_IP = os.environ.get("ARM_IP", "192.168.1.190")
SAMPLE_PERIOD_SEC = 0.5
JOINT_K = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0,2.0]
JOINT_D = [0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6]


class DragClient(Node):
    def __init__(self):
        super().__init__("drag_arm_b_test")
        self.connect_cli = self.create_client(Connect, "/arm/connect_robot")
        self.clear_err_cli = self.create_client(
            ArmClearErr, "/arm/clear_arm_error"
        )
        self.log_cli = self.create_client(LogSwitch, "/arm/log_switch")
        self.state_cli = self.create_client(SetArmState, "/arm/set_arm_state")
        self.impedance_cli = self.create_client(
            SetImpedanceType, "/arm/set_impedance_type"
        )
        self.joint_kd_cli = self.create_client(
            SetJointKD, "/arm/set_joint_kd"
        )
        self.drag_cli = self.create_client(
            SetDragSpace, "/arm/set_drag_space"
        )
        self.data_cli = self.create_client(RobotData, "/arm/get_robot_data")

    def call(self, client, request, allow_already_connected=False):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for service {client.srv_name} ...")

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.exception() is not None:
            raise RuntimeError(
                f"{client.srv_name} call raised: {future.exception()}"
            )

        response = future.result()
        if hasattr(response, "success") and not response.success:
            already_connected = (
                allow_already_connected
                and "Already connected" in response.message
            )
            if not already_connected:
                raise RuntimeError(
                    f"{client.srv_name} failed: {response.message}"
                )
        return response


def command(client, service_client, service_type, **fields):
    request = service_type.Request()
    for name, value in fields.items():
        setattr(request, name, value)
    return client.call(service_client, request)


def robot_record(client):
    response = client.call(client.data_cli, RobotData.Request())
    data = response.data
    if (
        len(data.states) <= ARM_INDEX
        or len(data.inputs) <= ARM_INDEX
        or len(data.outputs) <= ARM_INDEX
    ):
        raise RuntimeError("No B-arm data in robot feedback")

    state = data.states[ARM_INDEX]
    input_data = data.inputs[ARM_INDEX]
    output = data.outputs[ARM_INDEX]
    return {
        "cur_state": state.cur_state,
        "imp_type": input_data.imp_type,
        "drag_sp_type": input_data.drag_sp_type,
        "frame_serial": output.out_frame_serial,
        "fb_joint_pos": list(output.fb_joint_pos),
        "fb_joint_c_toq": list(output.fb_joint_c_toq),
        "fb_joint_s_toq": list(output.fb_joint_s_toq),
        "fb_joint_them": list(output.fb_joint_them),
        "est_joint_firc": list(output.est_joint_firc),
        "est_joint_firc_dot": list(output.est_joint_firc_dot),
        "est_joint_force": list(output.est_joint_force),
        "est_cart_fn": list(output.est_cart_fn),
    }


def save_csv(records):
    output_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(
        output_dir,
        f"drag_joint_{ARM_ID}_{datetime.now():%Y%m%d_%H%M%S}.csv",
    )
    with open(output_path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)
    return output_path


def main(args=None):
    rclpy.init(args=args)
    client = DragClient()
    logger = client.get_logger()
    records = []
    drag_enabled = False
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        connect_request = Connect.Request()
        connect_request.robot_ip = ROBOT_IP
        client.call(
            client.connect_cli,
            connect_request,
            allow_already_connected=True,
        )

        command(
            client,
            client.clear_err_cli,
            ArmClearErr,
            arm_id=ARM_ID,
        )
        command(
            client,
            client.log_cli,
            LogSwitch,
            global_or_local="global",
            flag="1",
        )
        command(
            client,
            client.log_cli,
            LogSwitch,
            global_or_local="local",
            flag="1",
        )
        command(
            client,
            client.joint_kd_cli,
            SetJointKD,
            arm_id=ARM_ID,
            k=JOINT_K,
            d=JOINT_D,
        )
        time.sleep(0.5)
        command(
            client,
            client.state_cli,
            SetArmState,
            arm_id=ARM_ID,
            state=3,
        )
        time.sleep(0.5)
        command(
            client,
            client.impedance_cli,
            SetImpedanceType,
            arm_id=ARM_ID,
            type=1,
        )
        time.sleep(0.5)
        command(
            client,
            client.drag_cli,
            SetDragSpace,
            arm_id=ARM_ID,
            dg_type=1,
        )
        drag_enabled = True
        time.sleep(0.3)

        status = robot_record(client)
        if (
            status["cur_state"] != 3
            or status["imp_type"] != 1
            or status["drag_sp_type"] != 1
        ):
            raise RuntimeError(
                "Unexpected drag-mode feedback: "
                f"state={status['cur_state']}, "
                f"impedance={status['imp_type']}, "
                f"drag={status['drag_sp_type']}"
            )

        logger.info("B arm is in joint drag mode. Manually guide the arm.")
        logger.info("Recording at 2 Hz. Press Ctrl+C to stop.")
        start_time = time.monotonic()
        while not stop_requested:
            loop_start = time.monotonic()
            record = robot_record(client)
            record = {
                "timestamp": round(loop_start - start_time, 3),
                **record,
            }
            records.append(record)
            logger.info(
                f"[{record['timestamp']:.1f}s] "
                f"Joint Pos: {[round(x, 2) for x in record['fb_joint_pos']]}"
            )
            sleep_time = SAMPLE_PERIOD_SEC - (time.monotonic() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)
    except Exception as exc:
        logger.error(str(exc))
    finally:
        if drag_enabled:
            try:
                command(
                    client,
                    client.drag_cli,
                    SetDragSpace,
                    arm_id=ARM_ID,
                    dg_type=0,
                )
            except Exception as exc:
                logger.error(f"Exit drag modefailed: {exc}")
        try:
            command(
                client,
                client.state_cli,
                SetArmState,
                arm_id=ARM_ID,
                state=0,
            )
        except Exception as exc:
            logger.error(f"Disable B armfailed: {exc}")

        # if records:
        #     try:
        
        
        #     except Exception as exc:
        

        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
