"""Top-down rendering of a World as an RGB image.

Uses the Agg backend directly instead of pyplot: no global figure state, no GUI
dependency, and figures can be reused between frames.
"""

from __future__ import annotations

import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Rectangle

from ._robotsim import World

_ROBOT_COLOR = "#1f77b4"
_GOAL_COLOR = "#2ca02c"
_OBSTACLE_COLOR = "#555555"
_LIDAR_COLOR = "#d62728"
_TRAIL_COLOR = "#1f77b4"


class Renderer:
    """Draws frames of one World. Reuse an instance to render many frames.

    The trail is built from the poses seen by ``frame()``, so it only shows the
    steps that were actually rendered. Call ``frame()`` every step to get a
    continuous trail, and ``clear_trail()`` when a new episode starts.
    """

    def __init__(
        self, world: World, size_px: int = 480, dpi: int = 100, show_legend: bool = True
    ) -> None:
        self._world = world
        self._show_legend = show_legend
        cfg = world.config
        aspect = cfg.map_height / cfg.map_width
        self._figure = Figure(figsize=(size_px / dpi, size_px * aspect / dpi), dpi=dpi)
        self._canvas = FigureCanvasAgg(self._figure)
        self._axes = self._figure.add_axes((0.0, 0.0, 1.0, 1.0))
        self._trail: list[tuple[float, float]] = []

    def clear_trail(self) -> None:
        self._trail.clear()

    def frame(self, show_lidar: bool = True, show_trail: bool = True) -> np.ndarray:
        """Renders the current state and returns an (H, W, 3) uint8 array."""
        world = self._world
        cfg = world.config
        axes = self._axes
        axes.clear()
        axes.set_xlim(0.0, cfg.map_width)
        axes.set_ylim(0.0, cfg.map_height)
        axes.set_aspect("equal")
        axes.set_xticks([])
        axes.set_yticks([])

        # Interior obstacles only: the four boundary walls lie outside the axes.
        for x_lo, y_lo, x_hi, y_hi in cfg.obstacles:
            axes.add_patch(
                Rectangle((x_lo, y_lo), x_hi - x_lo, y_hi - y_lo, color=_OBSTACLE_COLOR)
            )

        x, y = world.position
        heading = world.heading

        if show_lidar:
            angles = heading + np.arange(cfg.lidar_beams) * (2.0 * np.pi / cfg.lidar_beams)
            ranges = world.lidar()
            axes.plot(
                np.column_stack([np.full_like(ranges, x), x + ranges * np.cos(angles)]).T,
                np.column_stack([np.full_like(ranges, y), y + ranges * np.sin(angles)]).T,
                color=_LIDAR_COLOR,
                linewidth=0.5,
                alpha=0.35,
            )

        if show_trail:
            self._trail.append((x, y))
            if len(self._trail) > 1:
                trail = np.asarray(self._trail)
                axes.plot(trail[:, 0], trail[:, 1], color=_TRAIL_COLOR, linewidth=1.0, alpha=0.6)

        goal_x, goal_y = world.goal
        axes.add_patch(
            Circle((goal_x, goal_y), cfg.goal_radius, color=_GOAL_COLOR, alpha=0.6)
        )
        axes.add_patch(Circle((x, y), cfg.robot_radius, color=_ROBOT_COLOR))
        axes.arrow(
            x,
            y,
            cfg.robot_radius * 1.6 * np.cos(heading),
            cfg.robot_radius * 1.6 * np.sin(heading),
            head_width=cfg.robot_radius * 0.8,
            color="black",
            length_includes_head=True,
        )
        if self._show_legend:
            handles = [
                Line2D([], [], marker="o", color=_ROBOT_COLOR, linestyle="", label="robot"),
                Line2D([], [], marker="o", color=_GOAL_COLOR, linestyle="", label="goal"),
                Line2D([], [], color=_LIDAR_COLOR, alpha=0.6, label="LiDAR"),
                Line2D([], [], color=_TRAIL_COLOR, label="path"),
                Line2D([], [], marker="s", color=_OBSTACLE_COLOR, linestyle="",
                       label="obstacle"),
            ]
            axes.legend(
                handles=handles,
                loc="lower left",
                fontsize=7,
                framealpha=0.85,
                handlelength=1.2,
                borderpad=0.4,
                labelspacing=0.3,
            )

        axes.text(
            0.02,
            0.98,
            f"step {world.step_count}",
            transform=axes.transAxes,
            va="top",
            fontsize=8,
        )

        self._canvas.draw()
        rgba = np.asarray(self._canvas.buffer_rgba())
        return rgba[..., :3].copy()