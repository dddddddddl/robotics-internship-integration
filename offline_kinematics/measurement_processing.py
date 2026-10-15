"""Measurement data processing converted from the two MATLAB data scripts."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Iterable

import numpy as np
import pandas as pd


@dataclass
class CalibrationResult:
    session: str
    normal: np.ndarray
    origin: np.ndarray
    radius: float
    z_mean: float
    z_var: float


def fit_plane_normal(points: np.ndarray) -> np.ndarray:
    centroid = points.mean(axis=0)
    _, _, vh = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vh[-1, :]
    if normal[2] < 0:
        normal = -normal
    return normal


def fit_circle_2d(a: Iterable[float], b: Iterable[float]) -> tuple[float, float, float]:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    A = np.column_stack((2 * a, 2 * b, np.ones_like(a)))
    B = a**2 + b**2
    params, *_ = np.linalg.lstsq(A, B, rcond=None)
    c1, c2, C = params
    radius = np.sqrt(C + c1**2 + c2**2)
    return float(c1), float(c2), float(radius)


def process_head_calibration_excel(
    file_path: str,
    sheet_name: str = "Sheet1",
    session_col: str = "测量次数",
    part_col: str = "测量部位",
    plane_part: str = "安装平面",
    cylinder_part: str = "圆柱面",
    x_col: str = "实际X",
    y_col: str = "实际Y",
    z_col: str = "实际Z",
) -> list[CalibrationResult]:
    data = pd.read_excel(file_path, sheet_name=sheet_name)
    data = data.rename(columns={col: str(col).replace(" ", "") for col in data.columns})
    if part_col not in data.columns and "部位" in data.columns:
        part_col = "部位"
    missing = [col for col in (session_col, part_col, x_col, y_col, z_col) if col not in data.columns]
    if missing:
        raise KeyError(f"missing columns: {missing}; available columns: {list(data.columns)}")

    for col in (x_col, y_col, z_col):
        data[col] = pd.to_numeric(data[col], errors="coerce")

    results: list[CalibrationResult] = []

    for session in data[session_col].drop_duplicates():
        curr = data[data[session_col] == session]
        plane_df = curr[curr[part_col] == plane_part][[x_col, y_col, z_col]].dropna()
        cyl_df = curr[curr[part_col] == cylinder_part][[x_col, y_col, z_col]].dropna()
        plane = plane_df.to_numpy(dtype=float)
        cyl = cyl_df.to_numpy(dtype=float)
        if len(plane) < 3:
            raise ValueError(f"{session}: at least 3 plane points are required")
        if len(cyl) < 3:
            raise ValueError(f"{session}: at least 3 cylinder points are required")

        z_mean = float(np.mean(plane[:, 2]))
        z_var = float(np.var(plane[:, 2], ddof=1))
        normal = fit_plane_normal(plane)
        xc, yc, radius = fit_circle_2d(cyl[:, 0], cyl[:, 1])
        origin = np.array([xc, yc, z_mean])
        results.append(CalibrationResult(str(session), normal, origin, radius, z_mean, z_var))

    return results


def demo_fit_yz_circle() -> tuple[float, float, float, float]:
    data = np.array(
        [
            [303.811, 553.130, -460.135],
            [303.811, 555.461, -461.765],
            [303.811, 559.845, -463.177],
            [303.811, 565.528, -462.562],
            [303.810, 570.073, -459.785],
            [303.811, 573.230, -454.826],
            [303.811, 573.706, -448.554],
            [303.811, 570.877, -442.635],
            [303.811, 566.743, -439.491],
            [303.811, 561.892, -438.303],
            [303.811, 555.459, -439.805],
            [303.811, 551.984, -442.584],
            [303.811, 549.510, -446.958],
            [303.811, 548.906, -450.829],
            [303.811, 549.892, -455.657],
        ]
    )
    x_mean = float(np.mean(data[:, 0]))
    yc, zc, radius = fit_circle_2d(data[:, 1], data[:, 2])
    return x_mean, yc, zc, radius


def main() -> None:
    parser = argparse.ArgumentParser(description="JIBOT measurement processing")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("head-excel", help="process head calibration Excel data")
    p.add_argument("file_path")
    p.add_argument("--sheet", default="Sheet1")
    p.set_defaults(command_func=lambda a: print_head_results(a.file_path, a.sheet))

    p = sub.add_parser("demo-circle", help="run the built-in Y/Z circle fitting demo")
    p.set_defaults(command_func=lambda a: print_demo_circle())

    args = parser.parse_args()
    args.command_func(args)


def print_head_results(file_path: str, sheet: str) -> None:
    results = process_head_calibration_excel(file_path, sheet_name=sheet)
    for r in results:
        print(f"\n--- {r.session} ---")
        print(f"normal: [{r.normal[0]:.6f}, {r.normal[1]:.6f}, {r.normal[2]:.6f}]")
        print(f"z_mean={r.z_mean:.4f} mm, z_var={r.z_var:.6e}")
        print(f"origin: [{r.origin[0]:.4f}, {r.origin[1]:.4f}, {r.origin[2]:.4f}], radius={r.radius:.4f} mm")

    normals = np.vstack([r.normal for r in results])
    origins = np.vstack([r.origin for r in results])
    print("\n=== final average calibration ===")
    print("normal:", np.mean(normals, axis=0))
    print("origin:", np.mean(origins, axis=0))


def print_demo_circle() -> None:
    x, y, z, r = demo_fit_yz_circle()
    print("--- demo circle result ---")
    print(f"center X: {x:.3f}")
    print(f"center Y: {y:.3f}")
    print(f"center Z: {z:.3f}")
    print(f"radius: {r:.3f}")


if __name__ == "__main__":
    main()
