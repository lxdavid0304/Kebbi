from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np
import cv2

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
        occ_mask = (local_grid == OCCUPIED).astype(np.uint8)
        free_mask = (local_grid == FREE).astype(np.uint8)

        roi_center_local_x = (meta.roi_x_min + meta.roi_x_max) / 2.0
        roi_center_local_y = (meta.roi_y_min + meta.roi_y_max) / 2.0

        yaw_deg = pose.heading_deg
        yaw_rad = math.radians(yaw_deg)
        cos_h = math.cos(yaw_rad)
        sin_h = math.sin(yaw_rad)

        roi_center_world_x = pose.x_m + roi_center_local_x * cos_h - roi_center_local_y * sin_h
        roi_center_world_y = pose.y_m + roi_center_local_x * sin_h + roi_center_local_y * cos_h

        center_x = cols / 2.0
        center_y = rows / 2.0
        rot_mat = cv2.getRotationMatrix2D((center_x, center_y), yaw_deg, 1.0)
        rotated_occ = cv2.warpAffine(
            occ_mask,
            rot_mat,
            (cols, rows),
            flags=cv2.INTER_NEAREST,
            borderValue=0,
        )
        rotated_free = cv2.warpAffine(
            free_mask,
            rot_mat,
            (cols, rows),
            flags=cv2.INTER_NEAREST,
            borderValue=0,
        )

        cx = (roi_center_world_x - (-cfg.WORLD_M / 2.0)) / self.cell_m
        cy = ((cfg.WORLD_M / 2.0) - roi_center_world_y) / self.cell_m
        offset_col = int(round(cx - cols / 2.0))
        offset_row = int(round(cy - rows / 2.0))

        self._blit_mask(rotated_free, offset_row, offset_col, mark_state=FREE, skip_state=OCCUPIED)
        self._blit_mask(rotated_occ, offset_row, offset_col, mark_state=OCCUPIED)


    def _world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        c = int((x - (-cfg.WORLD_M / 2)) / self.cell_m)
        r = int(((cfg.WORLD_M / 2) - y) / self.cell_m)
        return r, c

    def _blit_mask(self, mask: np.ndarray, offset_row: int, offset_col: int,
                   *, mark_state: int, skip_state: int | None = None):
        if mask is None or mask.size == 0:
            return
        rows, cols = mask.shape
        r0 = max(0, offset_row)
        c0 = max(0, offset_col)
        r1 = min(self.grid_size, offset_row + rows)
        c1 = min(self.grid_size, offset_col + cols)
        if r0 >= r1 or c0 >= c1:
            return
        sub_mask = mask[r0 - offset_row : r1 - offset_row, c0 - offset_col : c1 - offset_col]
        if sub_mask.size == 0:
            return
        target = self.grid[r0:r1, c0:c1]
        valid = sub_mask > 0
        if skip_state is not None:
            valid &= (target != skip_state)
        target[valid] = mark_state

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
