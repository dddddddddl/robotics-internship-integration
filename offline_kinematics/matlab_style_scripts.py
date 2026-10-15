"""Script-style entry points matching the original MATLAB files."""

from __future__ import annotations

import numpy as np

from jibot_kinematics import (
    all_frames,
    body_to_camera,
    body_to_lidar360,
    camera_to_ground,
    left_arm_angles,
    left_arm_chain,
    leg_to_body,
    lidar360_to_ground,
    object_to_ground,
    print_transform,
    right_arm_angles,
    right_arm_chain,
    transl,
)
from jibot_plot import plot_points_and_frames


DEFAULT_LEG_ANGLES_DEG = (12.192996, 0.437391, -12.630388, 0.0)
DEFAULT_LEFT_ARM_ANGLES_DEG = (140.233668, 63.403568, -14.56237, -82.271734, 5.520698, 18.978728, 48.376372)
DEFAULT_RIGHT_ARM_ANGLES_DEG = (-13.98, -31.67, 8.78, -78.06, 31.63, -6.89, -13.85)
DEFAULT_HEAD_ANGLES_DEG = (0.0, 0.0)


def print_transforms(transforms: dict, names: list[str]) -> None:
    for name in names:
        value = transforms[name]
        if isinstance(value, np.ndarray) and value.shape == (4, 4):
            print_transform(name, value)
        else:
            print(f"\n--- {name} ---")
            print(value)


def run_body2ground(q_deg=DEFAULT_LEG_ANGLES_DEG, plot=True) -> None:
    transforms = leg_to_body(q_deg, unit="deg")
    print_transforms(transforms, ["T_ground_leg_base", "T_ground_M1", "T_ground_M2", "T_ground_M3", "T_ground_body"])
    p = transl(transforms["T_ground_body"])
    print(f"\nBody position: X={p[0]:.6f}, Y={p[1]:.6f}, Z={p[2]:.6f} m")
    if plot:
        plot_points_and_frames(
            transforms,
            ["ground", "T_ground_leg_base", "T_ground_M1", "T_ground_M2", "T_ground_M3", "T_ground_body"],
            "JIBOT leg/body frame chain",
            [
                ("ground", "T_ground_leg_base"),
                ("T_ground_leg_base", "T_ground_M1"),
                ("T_ground_M1", "T_ground_M2"),
                ("T_ground_M2", "T_ground_M3"),
                ("T_ground_M3", "T_ground_body"),
            ],
        )


def run_cam2body(q_head_deg=DEFAULT_HEAD_ANGLES_DEG, plot=True) -> None:
    transforms = body_to_camera(q_head_deg, unit="deg")
    print_transforms(transforms, ["T_body_yaw", "T_body_pitch", "T_body_camera"])
    p = transl(transforms["T_body_camera"])
    print(f"\nCamera in body: X={p[0]:.6f}, Y={p[1]:.6f}, Z={p[2]:.6f} m")
    if plot:
        plot_points_and_frames(
            transforms,
            ["ground", "T_body_yaw", "T_body_pitch", "T_body_camera"],
            "JIBOT body to camera frame chain",
            [("ground", "T_body_yaw"), ("T_body_yaw", "T_body_pitch"), ("T_body_pitch", "T_body_camera")],
        )


def run_cam2ground(q_leg_deg=DEFAULT_LEG_ANGLES_DEG, q_head_deg=DEFAULT_HEAD_ANGLES_DEG, plot=True) -> None:
    transforms = camera_to_ground(q_leg_deg, q_head_deg, unit="deg")
    print_transforms(transforms, ["T_ground_body", "T_body_yaw", "T_body_pitch", "T_body_camera", "T_ground_yaw", "T_ground_pitch", "T_ground_camera"])
    p = transl(transforms["T_ground_camera"])
    print(f"\nCamera position in ground frame: X={p[0]:.6f}, Y={p[1]:.6f}, Z={p[2]:.6f} m")
    if plot:
        plot_points_and_frames(
            transforms,
            [
                "ground",
                "T_ground_leg_base",
                "T_ground_M1",
                "T_ground_M2",
                "T_ground_M3",
                "T_ground_body",
                "T_ground_yaw",
                "T_ground_pitch",
                "T_ground_camera",
            ],
            "JIBOT camera to ground frame chain",
            [
                ("ground", "T_ground_leg_base"),
                ("T_ground_leg_base", "T_ground_M1"),
                ("T_ground_M1", "T_ground_M2"),
                ("T_ground_M2", "T_ground_M3"),
                ("T_ground_M3", "T_ground_body"),
                ("T_ground_body", "T_ground_yaw"),
                ("T_ground_yaw", "T_ground_pitch"),
                ("T_ground_pitch", "T_ground_camera"),
            ],
        )


def run_lidar360_frame(plot=True) -> None:
    transforms = body_to_lidar360()
    print_transforms(transforms, ["T_body_lidar360"])
    p = transl(transforms["T_body_lidar360"])
    print(f"\nLidar360 in body: X={p[0]:.6f}, Y={p[1]:.6f}, Z={p[2]:.6f} m")
    if plot:
        plot_points_and_frames(
            transforms,
            ["ground", "T_body_lidar360"],
            "JIBOT body to lidar360 frame",
            [("ground", "T_body_lidar360")],
        )


def run_lidar360_to_ground(q_deg=DEFAULT_LEG_ANGLES_DEG, plot=True) -> None:
    transforms = lidar360_to_ground(q_deg, unit="deg")
    print_transforms(transforms, ["T_ground_body", "T_body_lidar360", "T_ground_lidar360"])
    p = transl(transforms["T_ground_lidar360"])
    print(f"\nLidar360 in ground: X={p[0]:.6f}, Y={p[1]:.6f}, Z={p[2]:.6f} m")
    if plot:
        plot_points_and_frames(
            transforms,
            ["ground", "T_ground_leg_base", "T_ground_M1", "T_ground_M2", "T_ground_M3", "T_ground_body", "T_ground_lidar360"],
            "JIBOT lidar360 to ground frame chain",
            [
                ("ground", "T_ground_leg_base"),
                ("T_ground_leg_base", "T_ground_M1"),
                ("T_ground_M1", "T_ground_M2"),
                ("T_ground_M2", "T_ground_M3"),
                ("T_ground_M3", "T_ground_body"),
                ("T_ground_body", "T_ground_lidar360"),
            ],
        )


def run_object2ground(q_leg_deg=DEFAULT_LEG_ANGLES_DEG, q_head_deg=DEFAULT_HEAD_ANGLES_DEG, object_xyz_cam=(0.30, 0.00, 0.80), plot=True) -> None:
    transforms = object_to_ground(object_xyz_cam, q_leg_deg, q_head_deg, unit="deg")
    print_transforms(transforms, ["T_ground_body", "T_body_camera", "T_ground_camera", "object_xyz_cam", "object_xyz_ground", "T_ground_object"])
    p = transforms["object_xyz_ground"]
    print(f"\nObject position in ground frame: X={p[0]:.6f}, Y={p[1]:.6f}, Z={p[2]:.6f} m")
    if plot:
        plot_points_and_frames(
            transforms,
            [
                "ground",
                "T_ground_leg_base",
                "T_ground_M1",
                "T_ground_M2",
                "T_ground_M3",
                "T_ground_body",
                "T_ground_yaw",
                "T_ground_pitch",
                "T_ground_camera",
                "T_ground_object",
            ],
            "JIBOT object point in ground frame",
            [
                ("ground", "T_ground_leg_base"),
                ("T_ground_leg_base", "T_ground_M1"),
                ("T_ground_M1", "T_ground_M2"),
                ("T_ground_M2", "T_ground_M3"),
                ("T_ground_M3", "T_ground_body"),
                ("T_ground_body", "T_ground_yaw"),
                ("T_ground_yaw", "T_ground_pitch"),
                ("T_ground_pitch", "T_ground_camera"),
                ("T_ground_camera", "T_ground_object"),
            ],
        )


def run_test_jibot_cam2ground(plot=True) -> None:
    run_object2ground(object_xyz_cam=(-0.16, 0.02, 0.98), plot=plot)


def run_left_arm(q_deg=DEFAULT_LEFT_ARM_ANGLES_DEG, plot=True) -> None:
    chain = left_arm_chain()
    q = left_arm_angles(q_deg)
    transforms = {}
    for i, T in enumerate(chain.joint_transforms(q, 7), start=1):
        transforms[f"T_body_L_arm_J{i}"] = T
        print_transform(f"T_body_L_arm_J{i}", T)
    T_end = chain.fkine(q)
    transforms["T_body_L_arm_TCP"] = T_end
    print_transform("T_body_L_arm_TCP", T_end)
    p = transl(T_end)
    print(f"\nLeft arm TCP in body: X={p[0]:.6f}, Y={p[1]:.6f}, Z={p[2]:.6f} m")
    if plot:
        names = ["ground", *[f"T_body_L_arm_J{i}" for i in range(1, 8)], "T_body_L_arm_TCP"]
        segments = [("ground", "T_body_L_arm_J1"), *[(f"T_body_L_arm_J{i}", f"T_body_L_arm_J{i + 1}") for i in range(1, 7)], ("T_body_L_arm_J7", "T_body_L_arm_TCP")]
        plot_points_and_frames(transforms, names, "JIBOT left arm in body frame", segments)


def run_right_arm(q_deg=DEFAULT_RIGHT_ARM_ANGLES_DEG, plot=True) -> None:
    chain = right_arm_chain()
    q = right_arm_angles(q_deg)
    transforms = {}
    for i, T in enumerate(chain.joint_transforms(q, 7), start=1):
        transforms[f"T_body_R_arm_J{i}"] = T
        print_transform(f"T_body_R_arm_J{i}", T)
    T_end = chain.fkine(q)
    transforms["T_body_R_arm_TCP"] = T_end
    print_transform("T_body_R_arm_TCP", T_end)
    p = transl(T_end)
    print(f"\nRight arm TCP in body: X={p[0]:.6f}, Y={p[1]:.6f}, Z={p[2]:.6f} m")
    if plot:
        names = ["ground", *[f"T_body_R_arm_J{i}" for i in range(1, 8)], "T_body_R_arm_TCP"]
        segments = [("ground", "T_body_R_arm_J1"), *[(f"T_body_R_arm_J{i}", f"T_body_R_arm_J{i + 1}") for i in range(1, 7)], ("T_body_R_arm_J7", "T_body_R_arm_TCP")]
        plot_points_and_frames(transforms, names, "JIBOT right arm in body frame", segments)


def run_all_frames(
    q_leg_deg=DEFAULT_LEG_ANGLES_DEG,
    q_left_deg=DEFAULT_LEFT_ARM_ANGLES_DEG,
    q_right_deg=DEFAULT_RIGHT_ARM_ANGLES_DEG,
    q_head_deg=DEFAULT_HEAD_ANGLES_DEG,
    plot=True,
) -> None:
    transforms = all_frames(q_leg_deg, q_left_deg, q_right_deg, q_head_deg)
    names = [
        "T_ground_leg_base",
        "T_ground_M1",
        "T_ground_M2",
        "T_ground_M3",
        "T_ground_body",
        *[f"T_ground_L_arm_J{i}" for i in range(1, 8)],
        "T_ground_L_arm_TCP",
        *[f"T_ground_R_arm_J{i}" for i in range(1, 8)],
        "T_ground_R_arm_TCP",
        "T_ground_head_yaw",
        "T_ground_head_pitch",
        "T_ground_camera",
    ]
    print_transforms(transforms, names)
    if plot:
        plot_points_and_frames(
            transforms,
            ["ground", *names],
            "JIBOT all frames",
            [
                ("ground", "T_ground_leg_base"),
                ("T_ground_leg_base", "T_ground_M1"),
                ("T_ground_M1", "T_ground_M2"),
                ("T_ground_M2", "T_ground_M3"),
                ("T_ground_M3", "T_ground_body"),
                ("T_ground_body", "T_ground_L_arm_J1"),
                *[(f"T_ground_L_arm_J{i}", f"T_ground_L_arm_J{i + 1}") for i in range(1, 7)],
                ("T_ground_L_arm_J7", "T_ground_L_arm_TCP"),
                ("T_ground_body", "T_ground_R_arm_J1"),
                *[(f"T_ground_R_arm_J{i}", f"T_ground_R_arm_J{i + 1}") for i in range(1, 7)],
                ("T_ground_R_arm_J7", "T_ground_R_arm_TCP"),
                ("T_ground_body", "T_ground_head_yaw"),
                ("T_ground_head_yaw", "T_ground_head_pitch"),
                ("T_ground_head_pitch", "T_ground_camera"),
            ],
            frame_length=0.10,
        )
