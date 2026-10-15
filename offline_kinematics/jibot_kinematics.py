"""JIBOT kinematics converted from the MATLAB scripts in the parent folder.

The code only depends on NumPy for calculation. Matplotlib is optional and is
used by the demo script for visualization.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Tuple

import numpy as np


ArrayLike = Iterable[float]


def transl(x: float | np.ndarray, y: float | None = None, z: float | None = None) -> np.ndarray:
    """Return a 4x4 translation matrix, or extract translation from a 4x4 matrix."""
    if y is None and z is None:
        arr = np.asarray(x, dtype=float)
        if arr.shape == (4, 4):
            return arr[:3, 3].copy()
        if arr.shape == (3,):
            x, y, z = arr
        else:
            raise ValueError("transl expects (x, y, z), a 3-vector, or a 4x4 matrix")
    T = np.eye(4)
    T[:3, 3] = [float(x), float(y), float(z)]
    return T


def trotx(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array(
        [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]],
        dtype=float,
    )


def troty(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array(
        [[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]],
        dtype=float,
    )


def trotz(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array(
        [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
        dtype=float,
    )


def t2r(T: np.ndarray) -> np.ndarray:
    return np.asarray(T, dtype=float)[:3, :3].copy()


@dataclass(frozen=True)
class ModifiedDHLink:
    d: float
    a: float
    alpha: float
    offset: float = 0.0

    def A(self, theta: float) -> np.ndarray:
        """Peter Corke Robotics Toolbox compatible modified-DH transform."""
        q = theta + self.offset
        return trotx(self.alpha) @ transl(self.a, 0.0, 0.0) @ trotz(q) @ transl(0.0, 0.0, self.d)


@dataclass(frozen=True)
class SerialChain:
    links: Tuple[ModifiedDHLink, ...]
    base: np.ndarray

    def fkine(self, q: ArrayLike, upto: int | None = None) -> np.ndarray:
        q_arr = list(q)
        n = len(self.links) if upto is None else upto
        if len(q_arr) < n:
            raise ValueError(f"expected at least {n} joint values, got {len(q_arr)}")
        T = np.array(self.base, dtype=float)
        for link, theta in zip(self.links[:n], q_arr[:n]):
            T = T @ link.A(theta)
        return T

    def joint_transforms(self, q: ArrayLike, count: int) -> List[np.ndarray]:
        return [self.fkine(q, upto=i) for i in range(1, count + 1)]


def mat_str(T: np.ndarray, decimals: int = 5) -> str:
    return np.array2string(
        np.asarray(T, dtype=float),
        formatter={"float_kind": lambda v: f"{v: .{decimals}f}"},
    )


def print_transform(name: str, T: np.ndarray, decimals: int = 5) -> None:
    print(f"\n--- {name} ---")
    print(mat_str(T, decimals=decimals))


def leg_to_body(q_leg: ArrayLike, unit: str = "deg", ground_y: float = 0.0) -> Dict[str, np.ndarray]:
    """Compute ground -> body and intermediate leg transforms.

    q_leg order: [M1, M2, M3, waist]. The MATLAB convention is q = -input.
    """
    q = np.asarray(q_leg, dtype=float)
    if q.shape != (4,):
        raise ValueError("q_leg must contain 4 values: [M1, M2, M3, waist]")
    if unit == "deg":
        q = np.deg2rad(q)
    elif unit != "rad":
        raise ValueError("unit must be 'deg' or 'rad'")
    q = -q

    T_ground_leg_base = transl(95.5 / 1000, ground_y, 332 / 1000) @ trotz(np.pi)

    P1 = np.array([0.0, -75.73, 100.57]) / 1000
    P2 = np.array([377.14, -75.75, 233.9638]) / 1000
    P3 = np.array([52.514, -74.09166667, 467.9568]) / 1000
    P_waist_rel = np.array([0.0, 74.09106667, 96.40175]) / 1000

    dP01 = P1
    dP12 = P2 - P1
    dP23 = P3 - P2

    T_ground_m1 = T_ground_leg_base @ transl(dP01) @ troty(q[0])
    T_ground_m2 = T_ground_m1 @ transl(dP12) @ troty(q[1])
    T_ground_m3 = T_ground_m2 @ transl(dP23) @ troty(q[2])

    T_waist_joint_axis = trotx(np.pi)
    T_ground_body = (
        T_ground_m3
        @ transl(P_waist_rel)
        @ T_waist_joint_axis
        @ trotz(q[3])
        @ T_waist_joint_axis
        @ trotz(np.pi)
    )

    return {
        "T_ground_leg_base": T_ground_leg_base,
        "T_ground_M1": T_ground_m1,
        "T_ground_M2": T_ground_m2,
        "T_ground_M3": T_ground_m3,
        "T_ground_body": T_ground_body,
    }


def left_arm_chain(base: np.ndarray | None = None) -> SerialChain:
    links = (
        ModifiedDHLink(0.1745, 0.0, -np.pi / 2),
        ModifiedDHLink(0.0, 0.0, np.pi / 2),
        ModifiedDHLink(0.287, 0.0, np.pi / 2),
        ModifiedDHLink(0.0, 0.018, np.pi / 2),
        ModifiedDHLink(0.314, 0.018, np.pi / 2),
        ModifiedDHLink(0.0, 0.0, np.pi / 2),
        ModifiedDHLink(0.0, 0.0, np.pi / 2),
        ModifiedDHLink(0.095, 0.0, -np.pi / 2),
    )
    if base is None:
        base = transl(0.0, 0.0506076, 335.8113 / 1000)
    return SerialChain(links, base)


def right_arm_chain(base: np.ndarray | None = None) -> SerialChain:
    links = (
        ModifiedDHLink(0.1745, 0.0, np.pi / 2),
        ModifiedDHLink(0.0, 0.0, np.pi / 2),
        ModifiedDHLink(0.287, 0.0, np.pi / 2),
        ModifiedDHLink(0.0, 0.018, np.pi / 2),
        ModifiedDHLink(0.314, 0.018, np.pi / 2),
        ModifiedDHLink(0.0, 0.0, np.pi / 2),
        ModifiedDHLink(0.0, 0.0, np.pi / 2),
        ModifiedDHLink(0.095, 0.0, np.pi / 2),
    )
    if base is None:
        base = transl(0.0, -0.0506076, 335.4138 / 1000)
    return SerialChain(links, base)


def left_arm_angles(q_deg: ArrayLike) -> np.ndarray:
    q = np.asarray(q_deg, dtype=float)
    if q.shape != (7,):
        raise ValueError("left arm needs 7 joint angles in degrees")
    return np.array(
        [
            np.deg2rad(q[0]),
            np.deg2rad(q[1]) - np.pi,
            np.deg2rad(q[2]) - np.pi,
            np.deg2rad(q[3]) - np.pi,
            np.deg2rad(q[4]) - np.pi,
            np.deg2rad(q[5]) + np.pi / 2,
            np.deg2rad(q[6]) - np.pi / 2,
            -np.pi / 2,
        ]
    )


def right_arm_angles(q_deg: ArrayLike) -> np.ndarray:
    q = np.asarray(q_deg, dtype=float)
    if q.shape != (7,):
        raise ValueError("right arm needs 7 joint angles in degrees")
    return np.array(
        [
            np.deg2rad(q[0]),
            np.deg2rad(q[1]) - np.pi,
            np.deg2rad(q[2]) - np.pi,
            np.deg2rad(q[3]) - np.pi,
            np.deg2rad(q[4]) - np.pi,
            np.deg2rad(q[5]) + np.pi / 2,
            np.deg2rad(q[6]) + np.pi / 2,
            np.pi / 2,
        ]
    )


def body_to_camera(q_head: ArrayLike = (0.0, 0.0), unit: str = "deg") -> Dict[str, np.ndarray]:
    q = np.asarray(q_head, dtype=float)
    if q.shape != (2,):
        raise ValueError("q_head must contain 2 values: [yaw, pitch]")
    if unit == "deg":
        q = np.deg2rad(q)
    elif unit != "rad":
        raise ValueError("unit must be 'deg' or 'rad'")

    q_yaw = q[0]
    q_pitch = q[1] + np.pi / 2
    q_static_roll = -np.pi / 2

    p_yaw_abs = np.array([0.624, 0.624, 402.02]) / 1000
    p_pitch_abs = np.array([0.58, -38.27, 478.9154]) / 1000
    p_cam_rel = np.array([75.41, 63.3, 73.96]) / 1000
    dp_yaw_pitch = p_pitch_abs - p_yaw_abs

    T_body_yaw = transl(p_yaw_abs) @ trotz(q_yaw)
    T_body_pitch = T_body_yaw @ transl(dp_yaw_pitch) @ trotx(-np.pi / 2) @ trotz(q_pitch)
    T_body_cam = (
        T_body_pitch
        @ transl(-p_cam_rel[2], -p_cam_rel[0], p_cam_rel[1])
        @ trotx(np.pi / 2)
        @ trotz(q_static_roll)
    )

    return {
        "T_body_yaw": T_body_yaw,
        "T_body_pitch": T_body_pitch,
        "T_body_camera": T_body_cam,
    }


def body_to_lidar360() -> Dict[str, np.ndarray]:
    p_body_lidar = np.array([128.0, 0.0, 286.50]) / 1000
    T_body_lidar = transl(p_body_lidar) @ trotz(-np.pi / 2) @ trotx(-np.pi / 2)
    R = t2r(T_body_lidar)
    if np.linalg.norm(R.T @ R - np.eye(3), ord="fro") >= 1e-12:
        raise ValueError("R_body_lidar is not orthogonal")
    if abs(np.linalg.det(R) - 1) >= 1e-12:
        raise ValueError("R_body_lidar is not right-handed")
    return {"T_body_lidar360": T_body_lidar}


def camera_to_ground(
    q_leg: ArrayLike = (0.0, 0.0, 0.0, 0.0),
    q_head: ArrayLike = (0.0, 0.0),
    unit: str = "rad",
) -> Dict[str, np.ndarray]:
    leg = leg_to_body(q_leg, unit=unit)
    head = body_to_camera(q_head, unit=unit)
    T_ground_body = leg["T_ground_body"]
    out = {**leg, **head}
    out["T_ground_yaw"] = T_ground_body @ head["T_body_yaw"]
    out["T_ground_pitch"] = T_ground_body @ head["T_body_pitch"]
    out["T_ground_camera"] = T_ground_body @ head["T_body_camera"]
    return out


def lidar360_to_ground(
    q_leg: ArrayLike = (0.0, 0.0, 0.0, 0.0),
    unit: str = "deg",
    ground_y: float = 3.1 / 1000,
) -> Dict[str, np.ndarray]:
    leg = leg_to_body(q_leg, unit=unit, ground_y=ground_y)
    lidar = body_to_lidar360()
    out = {**leg, **lidar}
    out["T_ground_lidar360"] = leg["T_ground_body"] @ lidar["T_body_lidar360"]
    return out


def object_to_ground(
    object_xyz_cam: ArrayLike,
    q_leg: ArrayLike = (0.0, 0.0, 0.0, 0.0),
    q_head: ArrayLike = (0.0, 0.0),
    unit: str = "rad",
) -> Dict[str, np.ndarray]:
    transforms = camera_to_ground(q_leg=q_leg, q_head=q_head, unit=unit)
    point_cam = np.r_[np.asarray(object_xyz_cam, dtype=float), 1.0]
    point_ground = transforms["T_ground_camera"] @ point_cam
    T_ground_object = transforms["T_ground_camera"].copy()
    T_ground_object[:3, 3] = point_ground[:3]
    transforms["object_xyz_cam"] = np.asarray(object_xyz_cam, dtype=float)
    transforms["object_xyz_ground"] = point_ground[:3]
    transforms["T_ground_object"] = T_ground_object
    return transforms


def all_frames(
    q_leg_deg: ArrayLike = (12.192996, 0.437391, -12.630388, 0.0),
    q_left_deg: ArrayLike = (140.233668, 63.403568, -14.56237, -82.271734, 5.520698, 18.978728, 48.376372),
    q_right_deg: ArrayLike = (-13.98, -31.67, 8.78, -78.06, 31.63, -6.89, -13.85),
    q_head_deg: ArrayLike = (0.0, 0.0),
) -> Dict[str, np.ndarray]:
    leg = leg_to_body(q_leg_deg, unit="deg")
    T_ground_body = leg["T_ground_body"]

    left_base = T_ground_body @ transl(0.0, 0.0506076, 335.8113 / 1000)
    right_base = T_ground_body @ transl(0.0, -0.0506076, 335.4138 / 1000)
    L_ARM = left_arm_chain(base=left_base)
    R_ARM = right_arm_chain(base=right_base)
    q_left = left_arm_angles(q_left_deg)
    q_right = right_arm_angles(q_right_deg)

    head = body_to_camera(q_head_deg, unit="deg")
    out = {**leg}
    out.update({f"T_ground_L_arm_J{i}": T for i, T in enumerate(L_ARM.joint_transforms(q_left, 7), start=1)})
    out["T_ground_L_arm_TCP"] = L_ARM.fkine(q_left)
    out.update({f"T_ground_R_arm_J{i}": T for i, T in enumerate(R_ARM.joint_transforms(q_right, 7), start=1)})
    out["T_ground_R_arm_TCP"] = R_ARM.fkine(q_right)
    out["T_ground_head_yaw"] = T_ground_body @ head["T_body_yaw"]
    out["T_ground_head_pitch"] = T_ground_body @ head["T_body_pitch"]
    out["T_ground_camera"] = T_ground_body @ head["T_body_camera"]
    return out

