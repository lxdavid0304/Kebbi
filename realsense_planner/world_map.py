from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np

from . import config as cfg
from .odometry import RobotPose

UNKNOWN = 2
FREE = 0
OCCUPIED = 1


@dataclass
class LocalMapMetadata:
    roi_x_min: float
    roi_x_max: float
    roi_y_min: float
    roi_y_max: float
    cell_m: float


class GlobalWorldMap:
    """
    維護固定 6x6 佔據圖，並可將局部 ROI 貼回世界座標。
    """

    def __init__(self, cell_m: float = cfg.CELL_M):
        self.cell_m = cell_m
        self.grid_size = int(cfg.WORLD_M / cell_m)
        self.grid = np.full((self.grid_size, self.grid_size), UNKNOWN, dtype=np.uint8)

    def reset(self):
        self.grid.fill(UNKNOWN)

    def integrate_local(self, local_grid: np.ndarray, meta: LocalMapMetadata, pose: RobotPose):
        if local_grid is None or local_grid.size == 0:
            return

        rows, cols = local_grid.shape
        x_vals = np.linspace(meta.roi_x_min + meta.cell_m / 2.0,
                             meta.roi_x_max - meta.cell_m / 2.0,
                             cols)
        y_vals = np.linspace(meta.roi_y_min + meta.cell_m / 2.0,
                             meta.roi_y_max - meta.cell_m / 2.0,
                             rows)

        sin_h = math.sin(math.radians(pose.heading_deg))
        cos_h = math.cos(math.radians(pose.heading_deg))

        for r_idx, y_local in enumerate(y_vals):
            for c_idx, x_local in enumerate(x_vals):
                state = local_grid[r_idx, c_idx]
                if state == UNKNOWN:
                    continue
                world_x = pose.x_m + x_local * cos_h - y_local * sin_h
                world_y = pose.y_m + x_local * sin_h + y_local * cos_h
                row, col = self._world_to_cell(world_x, world_y)
                if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
                    if state == OCCUPIED:
                        self.grid[row, col] = OCCUPIED
                    elif self.grid[row, col] == UNKNOWN:
                        self.grid[row, col] = FREE

    def _world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        c = int((x - (-cfg.WORLD_M / 2)) / self.cell_m)
        r = int(((cfg.WORLD_M / 2) - y) / self.cell_m)
        return r, c

    def coverage_ratio(self) -> float:
        total = self.grid.size
        unknown = np.count_nonzero(self.grid == UNKNOWN)
        return (total - unknown) / total if total > 0 else 0.0

    def to_planner_grid(self) -> np.ndarray:
        """
        產生 A* 可使用的二值格：0=可通行、1=障礙或未知。
        """
        planner_grid = np.ones_like(self.grid, dtype=np.uint8)
        planner_grid[self.grid == FREE] = 0
        return planner_grid
