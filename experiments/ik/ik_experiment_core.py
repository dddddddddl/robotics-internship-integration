#!/usr/bin/env python3
"""Pure geometry and data helpers for JIBOT arm IK experiments.

This module deliberately has no ROS dependency so trajectory/grid generation can
be tested on a development computer before running the robot-side client.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Iterator

import numpy as np


@dataclass(frozen=True)
class GridWaypoint:
    """One target in an X-Z workspace scan."""

    point_index: int
    x_index: int
    z_index: int
    target_matrix: np.ndarray


def validate_transform(matrix: np.ndarray, name: str = "transform", atol: float = 1e-5) -> np.ndarray:
    """Return a checked copy of a 4x4 right-handed homogeneous transform."""
    transform = np.asarray(matrix, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError(f"{name} must be 4x4, got {transform.shape}")
    if not np.all(np.isfinite(transform)):
        raise ValueError(f"{name} contains NaN or infinity")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=atol):
        raise ValueError(f"{name} bottom row must be [0, 0, 0, 1]")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=atol):
        raise ValueError(f"{name} rotation is not orthonormal")
    determinant = float(np.linalg.det(rotation))
    if not math.isclose(determinant, 1.0, abs_tol=atol):
        raise ValueError(f"{name} rotation determinant must be +1, got {determinant:.9g}")
    return transform.copy()


def transform_with_xyz(template: np.ndarray, xyz: Iterable[float]) -> np.ndarray:
    """Copy a transform while replacing only its translation."""
    transform = validate_transform(template, "template")
    position = np.asarray(list(xyz), dtype=float)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("xyz must contain three finite numbers")
    transform[:3, 3] = position
    return transform


def resolve_line_end_xyz(
    start_matrix: np.ndarray,
    end_xyz: Iterable[float] | None = None,
    delta_xyz: Iterable[float] | None = None,
) -> np.ndarray:
    """Resolve an absolute or start-relative Cartesian line endpoint."""
    start = validate_transform(start_matrix, "start_matrix")
    if (end_xyz is None) == (delta_xyz is None):
        raise ValueError("provide exactly one of end_xyz or delta_xyz")
    values = end_xyz if end_xyz is not None else delta_xyz
    resolved = np.asarray(list(values), dtype=float)
    if resolved.shape != (3,) or not np.all(np.isfinite(resolved)):
        raise ValueError("line endpoint values must contain three finite numbers")
    if delta_xyz is not None:
        resolved = start[:3, 3] + resolved
    return resolved


def resolve_line_steps(
    start_xyz: Iterable[float],
    end_xyz: Iterable[float],
    requested_steps: int = 0,
    max_step_m: float = 0.01,
) -> int:
    """Choose a waypoint count including both endpoints."""
    start = np.asarray(list(start_xyz), dtype=float)
    end = np.asarray(list(end_xyz), dtype=float)
    if start.shape != (3,) or end.shape != (3,):
        raise ValueError("start_xyz and end_xyz must each contain three numbers")
    if max_step_m <= 0:
        raise ValueError("max_step_m must be positive")
    distance = float(np.linalg.norm(end - start))
    minimum_steps = max(2, int(math.ceil(distance / max_step_m)) + 1)
    if requested_steps:
        if requested_steps < 2:
            raise ValueError("requested_steps must be 0 or at least 2")
        return max(requested_steps, minimum_steps)
    return minimum_steps


def generate_line_waypoints(
    start_matrix: np.ndarray,
    end_xyz: Iterable[float],
    requested_steps: int = 0,
    max_step_m: float = 0.01,
) -> list[np.ndarray]:
    """Generate a Cartesian straight line with orientation fixed at the start."""
    start = validate_transform(start_matrix, "start_matrix")
    end = np.asarray(list(end_xyz), dtype=float)
    if end.shape != (3,) or not np.all(np.isfinite(end)):
        raise ValueError("end_xyz must contain three finite numbers")
    count = resolve_line_steps(start[:3, 3], end, requested_steps, max_step_m)
    positions = np.linspace(start[:3, 3], end, count)
    return [transform_with_xyz(start, position) for position in positions]


def inclusive_axis_values(minimum: float, maximum: float, step: float) -> list[float]:
    """Return stable inclusive samples without cumulative floating-point drift."""
    if not all(math.isfinite(value) for value in (minimum, maximum, step)):
        raise ValueError("axis range values must be finite")
    if step <= 0:
        raise ValueError("axis step must be positive")
    if maximum < minimum:
        raise ValueError("axis maximum must be greater than or equal to minimum")
    span = maximum - minimum
    count = int(math.floor(span / step + 1e-10))
    values = [minimum + index * step for index in range(count + 1)]
    if not math.isclose(values[-1], maximum, abs_tol=max(1e-12, step * 1e-8)):
        values.append(maximum)
    else:
        values[-1] = maximum
    return values


def generate_xz_grid(
    template_matrix: np.ndarray,
    x_values: Iterable[float],
    z_values: Iterable[float],
    snake: bool = True,
) -> Iterator[GridWaypoint]:
    """Yield an X-Z grid at fixed Y and fixed orientation.

    Indices always refer to ascending X/Z axes. With ``snake=True``, every other
    X column is traversed in reverse Z order to reduce large target jumps.
    """
    template = validate_transform(template_matrix, "template_matrix")
    xs = [float(value) for value in x_values]
    zs = [float(value) for value in z_values]
    if not xs or not zs:
        raise ValueError("x_values and z_values must not be empty")
    if not all(math.isfinite(value) for value in [*xs, *zs]):
        raise ValueError("grid coordinates must be finite")

    point_index = 0
    for x_offset, x_value in enumerate(xs):
        z_order = range(len(zs))
        if snake and x_offset % 2:
            z_order = reversed(range(len(zs)))
        for z_offset in z_order:
            point_index += 1
            target = transform_with_xyz(
                template,
                [x_value, template[1, 3], zs[z_offset]],
            )
            yield GridWaypoint(
                point_index=point_index,
                x_index=x_offset + 1,
                z_index=z_offset + 1,
                target_matrix=target,
            )


def rotation_error_rad(target_rotation: np.ndarray, solved_rotation: np.ndarray) -> float:
    """Return the geodesic angle between two rotation matrices."""
    relative = np.asarray(target_rotation, dtype=float).T @ np.asarray(solved_rotation, dtype=float)
    cosine = float((np.trace(relative) - 1.0) / 2.0)
    return float(math.acos(max(-1.0, min(1.0, cosine))))


def pose_error(target: np.ndarray, solved: np.ndarray | None) -> tuple[float, float, float]:
    """Return position norm, orientation angle and max element error."""
    if solved is None:
        return float("inf"), float("inf"), float("inf")
    target_checked = validate_transform(target, "target")
    solved_checked = validate_transform(solved, "solved", atol=1e-3)
    position_error = float(np.linalg.norm(target_checked[:3, 3] - solved_checked[:3, 3]))
    orientation_error = rotation_error_rad(target_checked[:3, :3], solved_checked[:3, :3])
    matrix_error = float(np.max(np.abs(target_checked - solved_checked)))
    return position_error, orientation_error, matrix_error


def boundary_rows(results: Iterable[dict]) -> list[dict]:
    """Summarize the contiguous successful Z interval at each target X."""
    grouped: dict[float, list[float]] = {}
    all_targets: dict[float, list[float]] = {}
    for result in results:
        x_value = float(result["target_sensor_x_m"])
        z_value = float(result["target_sensor_z_m"])
        all_targets.setdefault(x_value, []).append(z_value)
        if not bool(result.get("success")):
            continue
        grouped.setdefault(x_value, []).append(z_value)

    summary = []
    for x_value in sorted(grouped):
        z_values = sorted(set(grouped[x_value]))
        gaps = [right - left for left, right in zip(z_values, z_values[1:])]
        target_values = sorted(set(all_targets[x_value]))
        target_gaps = [right - left for left, right in zip(target_values, target_values[1:])]
        nominal_step = min(target_gaps) if target_gaps else (min(gaps) if gaps else 0.0)
        has_internal_gap = any(gap > nominal_step * 1.5 for gap in gaps) if nominal_step > 0 else False
        summary.append(
            {
                "target_sensor_x_m": x_value,
                "success_count": len(z_values),
                "min_success_z_m": min(z_values),
                "max_success_z_m": max(z_values),
                "has_internal_gap": has_internal_gap,
            }
        )
    return summary
