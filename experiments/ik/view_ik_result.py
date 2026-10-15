#!/usr/bin/env python3
"""Visualize one recorded arm IK solution with the MATLAB-to-Python model."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path


def load_joint_row(csv_path: Path, point_index: int) -> tuple[dict, list[float]]:
    with csv_path.open("r", newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            if int(row["point_index"]) != point_index:
                continue
            split_columns = [row.get(f"joint_{index}_deg", "") for index in range(1, 8)]
            if all(value.strip() for value in split_columns):
                joints = [float(value) for value in split_columns]
            else:
                joints = [float(value) for value in json.loads(row["ik_joints_deg"])]
            if len(joints) != 7:
                raise ValueError(f"point {point_index} has {len(joints)} joints, expected 7")
            return row, joints
    raise KeyError(f"point_index={point_index} was not found in {csv_path}")


def parse_args():
    parser = argparse.ArgumentParser(description="View one IK CSV row using the converted JIBOT kinematics")
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--point-index", type=int, required=True)
    parser.add_argument("--arm", choices=["A", "B", "a", "b"], required=True)
    parser.add_argument("--no-plot", action="store_true", help="print FK only; do not open the 3D window")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    row, joints = load_joint_row(args.csv_path, args.point_index)
    model_dir = Path(__file__).resolve().parents[2] / "offline_kinematics"
    sys.path.insert(0, str(model_dir))
    from matlab_style_scripts import run_left_arm, run_right_arm

    print(f"point_index={args.point_index}, success={row.get('success')}, joints_deg={joints}")
    if args.arm.upper() == "A":
        run_left_arm(q_deg=joints, plot=not args.no_plot)
    else:
        run_right_arm(q_deg=joints, plot=not args.no_plot)


if __name__ == "__main__":
    main()
