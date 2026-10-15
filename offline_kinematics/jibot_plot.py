"""Small Matplotlib helpers for visualizing homogeneous frames."""

from __future__ import annotations

from typing import Iterable, Tuple

import numpy as np


def set_axes_equal(ax) -> None:
    limits = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    spans = limits[:, 1] - limits[:, 0]
    centers = np.mean(limits, axis=1)
    radius = 0.5 * max(spans)
    ax.set_xlim3d(centers[0] - radius, centers[0] + radius)
    ax.set_ylim3d(centers[1] - radius, centers[1] + radius)
    ax.set_zlim3d(centers[2] - radius, centers[2] + radius)


def draw_frame(ax, T: np.ndarray, name: str, length: float = 0.12) -> None:
    origin = T[:3, 3]
    R = T[:3, :3]
    colors = ("r", "g", "b")
    for i, color in enumerate(colors):
        axis = R[:, i] * length
        ax.quiver(
            origin[0],
            origin[1],
            origin[2],
            axis[0],
            axis[1],
            axis[2],
            color=color,
            linewidth=1.8,
        )
    ax.text(origin[0], origin[1], origin[2], f" {name}", fontsize=9)


def plot_points_and_frames(
    transforms: dict,
    names: Iterable[str],
    title: str,
    segments: Iterable[Tuple[str, str]] = (),
    frame_length: float = 0.12,
) -> None:
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title(title)
    ax.set_xlabel("X / m")
    ax.set_ylabel("Y / m")
    ax.set_zlabel("Z / m")
    ax.grid(True)
    ax.view_init(elev=25, azim=135)

    for name in names:
        T = np.eye(4) if name == "ground" else transforms[name]
        draw_frame(ax, T, name.replace("T_", ""), frame_length)
        p = T[:3, 3]
        ax.scatter(p[0], p[1], p[2], s=35)

    for a, b in segments:
        Ta = np.eye(4) if a == "ground" else transforms[a]
        Tb = np.eye(4) if b == "ground" else transforms[b]
        pa, pb = Ta[:3, 3], Tb[:3, 3]
        ax.plot([pa[0], pb[0]], [pa[1], pb[1]], [pa[2], pb[2]], linewidth=2)

    set_axes_equal(ax)
    plt.show()

