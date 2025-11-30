from __future__ import annotations

import os
from pathlib import Path


# World & grid configuration
WORLD_M = 6.0
CELL_M = 0.05
GRID_SIZE = int(WORLD_M / CELL_M)
ROI = {
    "x_min": -WORLD_M / 2,
    "x_max": WORLD_M / 2,
    "y_min": -WORLD_M / 2,
    "y_max": WORLD_M / 2,
}

# Actors & safety
HUMAN_START = (0.0, -1.5)
ROBOT_START = (0.0, -0.5)
ROBOT_INIT_YAW_DEG = 90.0
ROBOT_RADIUS_M = 0.20
SAFE_RADIUS_M = 0.35
BACKWARD_FAN_RADIUS_M = 1.20
BACKWARD_FAN_DEG = 60
BACKWARD_FAN_ALWAYS_RENDER = True
FAN_WRITE_TO_PLANNING_OCC = False
SEMICIRCLE_THETAS = 16
PATH_SAMPLE_DS = 0.15
PLACE6X6_LOG_RATE_SEC = 2.0

# Depth & point cloud filters
NEAR_M = 0.10
FAR_M_DEFAULT = 3.5
DEPTH_SCALE = 0.001
USE_ALIGNED_TO_DEPTH = True
GROUND_Z_RANGE = (0.05, 1.20)
GROUND_PLANE_RANSAC = {
    "enabled": True,
    "dist_thresh": 0.02,
    "num_iter": 2000,
    "prob": 0.999,
}

# Occupancy update
OCC_MIN_PTS = 3
OPEN_CLOSE_KERNEL = 3
INFLATE_CELLS = 2  # number of grid cells to dilate obstacles before planning
TEMPORAL_DECAY = 0.90

# Planning defaults
MAP_UPDATE_INTERVAL_SEC = 0.2
DRAW_GRID_ANNOTATION = False
A_STAR_START_WORLD = ROBOT_START
A_STAR_GOAL_WORLD = (0.0, 1.5)
POINTS_STRIDE = 1
APPLY_VIS_FREE_TO_PLANNING = True
HUMAN_DETECTION_MAX_DIST_MM=1200

# Robot TCP defaults
ROBOT_TCP_IP = "172.20.10.8"
ROBOT_TCP_PORT = 8888

# Debug toggles
PRINT_ENV_ON_IMPORT = True
DEBUG_CONVERSION = True

# Snapshot directory
DESKTOP_ENV = os.environ.get("DESKTOP")
if DESKTOP_ENV:
    SNAP_DIR_DEFAULT = Path(DESKTOP_ENV) / "occ_snaps"
else:
    SNAP_DIR_DEFAULT = Path.cwd() / "occ_snaps"


__all__ = [
    "WORLD_M",
    "CELL_M",
    "GRID_SIZE",
    "ROI",
    "HUMAN_START",
    "ROBOT_START",
    "ROBOT_INIT_YAW_DEG",
    "ROBOT_RADIUS_M",
    "SAFE_RADIUS_M",
    "BACKWARD_FAN_RADIUS_M",
    "BACKWARD_FAN_DEG",
    "BACKWARD_FAN_ALWAYS_RENDER",
    "FAN_WRITE_TO_PLANNING_OCC",
    "SEMICIRCLE_THETAS",
    "PATH_SAMPLE_DS",
    "PLACE6X6_LOG_RATE_SEC",
    "NEAR_M",
    "FAR_M_DEFAULT",
    "DEPTH_SCALE",
    "USE_ALIGNED_TO_DEPTH",
    "GROUND_Z_RANGE",
    "GROUND_PLANE_RANSAC",
    "OCC_MIN_PTS",
    "OPEN_CLOSE_KERNEL",
    "INFLATE_CELLS",
    "TEMPORAL_DECAY",
    "MAP_UPDATE_INTERVAL_SEC",
    "DRAW_GRID_ANNOTATION",
    "A_STAR_START_WORLD",
    "A_STAR_GOAL_WORLD",
    "POINTS_STRIDE",
    "APPLY_VIS_FREE_TO_PLANNING",
    "ROBOT_TCP_IP",
    "ROBOT_TCP_PORT",
    "PRINT_ENV_ON_IMPORT",
    "DEBUG_CONVERSION",
    "SNAP_DIR_DEFAULT",
]
