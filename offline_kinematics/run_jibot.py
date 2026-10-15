"""Command line demos corresponding to the converted MATLAB scripts."""

from __future__ import annotations

import argparse

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


def add_float_list(parser, name, nargs, default, help_text):
    parser.add_argument(name, nargs=nargs, type=float, default=default, help=help_text)


def print_selected(transforms: dict, names: list[str]) -> None:
    for name in names:
        value = transforms[name]
        if isinstance(value, np.ndarray) and value.shape == (4, 4):
            print_transform(name, value)
        else:
            print(f"\n--- {name} ---")
            print(value)


def cmd_body(args) -> None:
    t = leg_to_body(args.q_leg, unit=args.unit, ground_y=args.ground_y)
    print_selected(t, ["T_ground_leg_base", "T_ground_M1", "T_ground_M2", "T_ground_M3", "T_ground_body"])
    p = transl(t["T_ground_body"])
    print(f"\nBody position: X={p[0]:.6f}, Y={p[1]:.6f}, Z={p[2]:.6f} m")


def cmd_camera_body(args) -> None:
    t = body_to_camera(args.q_head, unit=args.unit)
    print_selected(t, ["T_body_yaw", "T_body_pitch", "T_body_camera"])
    p = transl(t["T_body_camera"])
    print(f"\nCamera in body: X={p[0]:.6f}, Y={p[1]:.6f}, Z={p[2]:.6f} m")


def cmd_camera_ground(args) -> None:
    t = camera_to_ground(args.q_leg, args.q_head, unit=args.unit)
    print_selected(t, ["T_ground_body", "T_body_camera", "T_ground_yaw", "T_ground_pitch", "T_ground_camera"])
    p = transl(t["T_ground_camera"])
    print(f"\nCamera in ground: X={p[0]:.6f}, Y={p[1]:.6f}, Z={p[2]:.6f} m")


def cmd_object(args) -> None:
    t = object_to_ground(args.object_xyz_cam, args.q_leg, args.q_head, unit=args.unit)
    print_selected(t, ["T_ground_body", "T_body_camera", "T_ground_camera", "object_xyz_cam", "object_xyz_ground", "T_ground_object"])
    p = t["object_xyz_ground"]
    print(f"\nObject in ground: X={p[0]:.6f}, Y={p[1]:.6f}, Z={p[2]:.6f} m")


def cmd_lidar_body(args) -> None:
    t = body_to_lidar360()
    print_selected(t, ["T_body_lidar360"])
    p = transl(t["T_body_lidar360"])
    print(f"\nLidar360 in body: X={p[0]:.6f}, Y={p[1]:.6f}, Z={p[2]:.6f} m")


def cmd_lidar_ground(args) -> None:
    t = lidar360_to_ground(args.q_leg, unit=args.unit, ground_y=args.ground_y)
    print_selected(t, ["T_ground_body", "T_body_lidar360", "T_ground_lidar360"])
    p = transl(t["T_ground_lidar360"])
    print(f"\nLidar360 in ground: X={p[0]:.6f}, Y={p[1]:.6f}, Z={p[2]:.6f} m")


def cmd_left_arm(args) -> None:
    chain = left_arm_chain()
    q = left_arm_angles(args.q_arm)
    for i, T in enumerate(chain.joint_transforms(q, 7), start=1):
        print_transform(f"T_body_L_arm_J{i}", T)
    T_end = chain.fkine(q)
    print_transform("T_body_L_arm_TCP", T_end)
    p = transl(T_end)
    print(f"\nLeft TCP in body: X={p[0]:.6f}, Y={p[1]:.6f}, Z={p[2]:.6f} m")


def cmd_right_arm(args) -> None:
    chain = right_arm_chain()
    q = right_arm_angles(args.q_arm)
    for i, T in enumerate(chain.joint_transforms(q, 7), start=1):
        print_transform(f"T_body_R_arm_J{i}", T)
    T_end = chain.fkine(q)
    print_transform("T_body_R_arm_TCP", T_end)
    p = transl(T_end)
    print(f"\nRight TCP in body: X={p[0]:.6f}, Y={p[1]:.6f}, Z={p[2]:.6f} m")


def cmd_all(args) -> None:
    t = all_frames(args.q_leg, args.q_left, args.q_right, args.q_head)
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
    print_selected(t, names)


def main() -> None:
    parser = argparse.ArgumentParser(description="JIBOT MATLAB-to-Python kinematics demos")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("body", help="jibot_body2ground.m")
    add_float_list(p, "--q-leg", 4, [11.64, -11.18, 6.28, 17.16], "leg angles")
    p.add_argument("--unit", choices=["deg", "rad"], default="deg")
    p.add_argument("--ground-y", type=float, default=0.0)
    p.set_defaults(func=cmd_body)

    p = sub.add_parser("camera-body", help="jibot_cam2body.m")
    add_float_list(p, "--q-head", 2, [0.0, 0.0], "head yaw/pitch")
    p.add_argument("--unit", choices=["deg", "rad"], default="deg")
    p.set_defaults(func=cmd_camera_body)

    p = sub.add_parser("camera-ground", help="jibot_cam2ground.m")
    add_float_list(p, "--q-leg", 4, [0.0, 0.0, 0.0, 0.0], "leg angles in degrees")
    add_float_list(p, "--q-head", 2, [0.0, 0.0], "head yaw/pitch in degrees")
    p.add_argument("--unit", choices=["deg", "rad"], default="deg")
    p.set_defaults(func=cmd_camera_ground)

    p = sub.add_parser("object-ground", help="jibot_object2ground.m / test_jibot_cam2ground.m")
    add_float_list(p, "--q-leg", 4, [0.0, 0.0, 0.0, 0.0], "leg angles in degrees")
    add_float_list(p, "--q-head", 2, [0.0, 0.0], "head yaw/pitch in degrees")
    add_float_list(p, "--object-xyz-cam", 3, [0.30, 0.00, 0.80], "object point in camera frame")
    p.add_argument("--unit", choices=["deg", "rad"], default="deg")
    p.set_defaults(func=cmd_object)

    p = sub.add_parser("lidar-body", help="jibot_lidar360_frame.m")
    p.set_defaults(func=cmd_lidar_body)

    p = sub.add_parser("lidar-ground", help="jibot_lidar360_to_ground.m")
    add_float_list(p, "--q-leg", 4, [0.0, 0.0, 0.0, 0.0], "leg angles")
    p.add_argument("--unit", choices=["deg", "rad"], default="deg")
    p.add_argument("--ground-y", type=float, default=3.1 / 1000)
    p.set_defaults(func=cmd_lidar_ground)

    p = sub.add_parser("left-arm", help="jibot_Larm2body.m")
    add_float_list(p, "--q-arm", 7, [31.458808, -66.508254, -51.083165, -31.341856, 13.96246, -18.133934, 8.254045], "left arm angles in degrees")
    p.set_defaults(func=cmd_left_arm)

    p = sub.add_parser("right-arm", help="jibot_Rarm2body.m")
    add_float_list(p, "--q-arm", 7, [-11.759217, -34.185804, 10.099768, -78.749281, 31.575269, -7.883864, -11.703297], "right arm angles in degrees")
    p.set_defaults(func=cmd_right_arm)

    p = sub.add_parser("all", help="jibot_all_fream.m")
    add_float_list(p, "--q-leg", 4, [12.192996, 0.437391, -12.630388, 0.0], "leg angles in degrees")
    add_float_list(p, "--q-left", 7, [140.233668, 63.403568, -14.56237, -82.271734, 5.520698, 18.978728, 48.376372], "left arm angles in degrees")
    add_float_list(p, "--q-right", 7, [-13.98, -31.67, 8.78, -78.06, 31.63, -6.89, -13.85], "right arm angles in degrees")
    add_float_list(p, "--q-head", 2, [0.0, 0.0], "head angles in degrees")
    p.set_defaults(func=cmd_all)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
