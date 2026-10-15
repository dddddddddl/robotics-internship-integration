#!/usr/bin/env python3
"""Shared ROS 2 service clients. All examples access the existing controller node to avoid concurrent native SDK connections. Set ARM_IP to override the vendor example address."""

import os

import rclpy
from rclpy.node import Node

from marvin_interfaces.srv import Connect, RobotData


#   export ARM_IP=192.168.1.190
ROBOT_IP = os.environ.get("ARM_IP", "192.168.1.190")


class MarvinServiceClient(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        
        self.connect_cli = self.create_client(Connect, "/arm/connect_robot")
        self.data_cli = self.create_client(RobotData, "/arm/get_robot_data")

    def call(self, client, request, allow_already_connected=False):
        """Call."""
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for {client.srv_name} ...")

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.exception() is not None:
            raise RuntimeError(
                f"{client.srv_name} raised: {future.exception()}"
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

    def connect_robot(self):
        """Connect robot."""
        request = Connect.Request()
        request.robot_ip = ROBOT_IP
        return self.call(
            self.connect_cli,
            request,
            allow_already_connected=True,
        )

    def robot_data(self):
        """Robot data."""
        return self.call(self.data_cli, RobotData.Request()).data


def command(node, client, service_type, **fields):
    """Command."""
    request = service_type.Request()
    for name, value in fields.items():
        setattr(request, name, value)
    return node.call(client, request)


def arm_index(arm_id):
    """Arm index."""
    arm = arm_id.upper()
    if arm == "A":
        return 0
    if arm == "B":
        return 1
    raise ValueError("ARM_ID must be 'A' or 'B'")


def finish(node):
    """Finish."""
    node.destroy_node()
    rclpy.shutdown()
