#!/usr/bin/env python3

from __future__ import annotations

import math
import csv
import tempfile
import unittest
from pathlib import Path

import numpy as np

from ik_experiment_core import (
    boundary_rows,
    generate_line_waypoints,
    generate_xz_grid,
    inclusive_axis_values,
    pose_error,
    resolve_line_end_xyz,
    resolve_line_steps,
    validate_transform,
)
from view_ik_result import load_joint_row


class IkExperimentCoreTest(unittest.TestCase):
    def test_validate_transform_rejects_reflection(self):
        matrix = np.eye(4)
        matrix[0, 0] = -1
        with self.assertRaisesRegex(ValueError, "determinant"):
            validate_transform(matrix)

    def test_line_includes_endpoints_and_honors_max_step(self):
        start = np.eye(4)
        start[:3, 3] = [1.0, 2.0, 3.0]
        waypoints = generate_line_waypoints(start, [1.025, 2.0, 3.0], max_step_m=0.01)
        self.assertEqual(len(waypoints), 4)
        np.testing.assert_allclose(waypoints[0], start)
        np.testing.assert_allclose(waypoints[-1][:3, 3], [1.025, 2.0, 3.0])
        self.assertTrue(all(np.allclose(item[:3, :3], np.eye(3)) for item in waypoints))

    def test_requested_line_steps_cannot_reduce_safety_sampling(self):
        self.assertEqual(resolve_line_steps([0, 0, 0], [0.1, 0, 0], 3, 0.01), 11)
        self.assertEqual(resolve_line_steps([0, 0, 0], [0.1, 0, 0], 21, 0.01), 21)

    def test_resolve_line_end_xyz_accepts_absolute_or_relative(self):
        start = np.eye(4)
        start[:3, 3] = [1.0, 2.0, 3.0]
        np.testing.assert_allclose(resolve_line_end_xyz(start, end_xyz=[4.0, 5.0, 6.0]), [4.0, 5.0, 6.0])
        np.testing.assert_allclose(resolve_line_end_xyz(start, delta_xyz=[0.1, -0.2, 0.3]), [1.1, 1.8, 3.3])
        with self.assertRaisesRegex(ValueError, "exactly one"):
            resolve_line_end_xyz(start)
        with self.assertRaisesRegex(ValueError, "exactly one"):
            resolve_line_end_xyz(start, end_xyz=[1, 2, 3], delta_xyz=[0, 0, 0])

    def test_inclusive_axis_values(self):
        self.assertEqual(inclusive_axis_values(-0.01, 0.01, 0.005), [-0.01, -0.005, 0.0, 0.004999999999999999, 0.01])

    def test_snake_grid_preserves_logical_indices(self):
        grid = list(generate_xz_grid(np.eye(4), [1.0, 2.0], [3.0, 4.0], snake=True))
        self.assertEqual([(p.x_index, p.z_index) for p in grid], [(1, 1), (1, 2), (2, 2), (2, 1)])
        self.assertEqual([p.target_matrix[2, 3] for p in grid], [3.0, 4.0, 4.0, 3.0])

    def test_pose_error(self):
        target = np.eye(4)
        solved = np.eye(4)
        solved[0, 3] = 0.003
        position, orientation, matrix = pose_error(target, solved)
        self.assertAlmostEqual(position, 0.003)
        self.assertAlmostEqual(orientation, 0.0)
        self.assertAlmostEqual(matrix, 0.003)

        rotated = np.eye(4)
        angle = math.radians(10)
        rotated[:3, :3] = [[math.cos(angle), -math.sin(angle), 0], [math.sin(angle), math.cos(angle), 0], [0, 0, 1]]
        _, orientation, _ = pose_error(target, rotated)
        self.assertAlmostEqual(orientation, angle)

    def test_boundary_rows(self):
        rows = boundary_rows(
            [
                {"success": True, "target_sensor_x_m": 1.0, "target_sensor_z_m": 2.0},
                {"success": False, "target_sensor_x_m": 1.0, "target_sensor_z_m": 2.1},
                {"success": True, "target_sensor_x_m": 1.0, "target_sensor_z_m": 2.2},
                {"success": True, "target_sensor_x_m": 1.0, "target_sensor_z_m": 2.4},
            ]
        )
        self.assertEqual(rows[0]["success_count"], 3)
        self.assertEqual(rows[0]["min_success_z_m"], 2.0)
        self.assertEqual(rows[0]["max_success_z_m"], 2.4)
        self.assertTrue(rows[0]["has_internal_gap"])

    def test_load_joint_row_from_split_columns(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "results.csv"
            fields = ["point_index", "success", *[f"joint_{index}_deg" for index in range(1, 8)]]
            with path.open("w", newline="", encoding="utf-8-sig") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                writer.writerow({"point_index": 3, "success": "True", **{f"joint_{index}_deg": index for index in range(1, 8)}})
            row, joints = load_joint_row(path, 3)
            self.assertEqual(row["success"], "True")
            self.assertEqual(joints, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])


if __name__ == "__main__":
    unittest.main()
