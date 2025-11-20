#!/usr/bin/env python3
"""
Streaming occupancy viewer that continuously applies the same safe-map
pipeline as capture_d435_pcd_safe_roi.py but only renders the processed
occupancy grid (with robot rectangle and backward safety fan). Point cloud
windows and raw views are intentionally omitted so that the display updates
in real time as obstacles change.
"""

from __future__ import annotations

import time

import cv2
import numpy as np
import open3d as o3d

from d435capture import config as cfg
from d435capture.occupancy import draw_backward_fan_reference
from d435capture.sensors import (
    auto_start_realsense,
    build_o3d_intrinsic_from_frame,
    frames_to_pointcloud_o3d,
    make_T_local_cam,
    transform_pcd,
)

ROI_X = (-1.5, 1.5)
ROI_Y = (0.0, 3.0)
Z_RANGE = (-2.0, 1.0)

WORLD_X = (-3.0, 3.0)
WORLD_Y = (-3.0, 3.0)
CELL_M = 0.05
GRID_W = int(round((WORLD_X[1] - WORLD_X[0]) / CELL_M))
GRID_H = int(round((WORLD_Y[1] - WORLD_Y[0]) / CELL_M))

CLOSE_KERNEL = 3
OPEN_KERNEL = 1
DILATE_KERNEL = (6, 5)

VOXEL_SIZE = 0.01
NB_NEIGHBORS = 30
STD_RATIO = 2.5
DEPTH_TRUNC = 3.5

CAM_YAW_DEG = 0.0
CAM_HEIGHT = 0.06
CAM_PITCH_DEG = -20.0


def _crop_roi(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    if not pcd.has_points():
        return pcd
    pts = np.asarray(pcd.points)
    mask = (
        (pts[:, 0] >= ROI_X[0])
        & (pts[:, 0] <= ROI_X[1])
        & (pts[:, 1] >= ROI_Y[0])
        & (pts[:, 1] <= ROI_Y[1])
        & (pts[:, 2] >= Z_RANGE[0])
        & (pts[:, 2] <= Z_RANGE[1])
    )
    idx = np.nonzero(mask)[0]
    return pcd.select_by_index(idx)


def _pcd_to_occ(pcd: o3d.geometry.PointCloud) -> np.ndarray:
    occ = np.full((GRID_H, GRID_W), 255, np.uint8)
    if not pcd.has_points():
        return occ
    pts = np.asarray(pcd.points)
    xs, ys, zs = pts[:, 0], pts[:, 1], pts[:, 2]
    mask = (zs >= Z_RANGE[0]) & (zs <= Z_RANGE[1])
    xs, ys = xs[mask], ys[mask]
    cols = ((xs - WORLD_X[0]) / CELL_M).astype(int)
    rows = ((WORLD_Y[1] - ys) / CELL_M).astype(int)
    valid = (rows >= 0) & (rows < GRID_H) & (cols >= 0) & (cols < GRID_W)
    rows, cols = rows[valid], cols[valid]
    counts = np.zeros_like(occ, dtype=np.int32)
    np.add.at(counts, (rows, cols), 1)
    occ[counts >= cfg.OCC_MIN_PTS] = 0
    return occ


def _apply_safe_filters(occ: np.ndarray) -> np.ndarray:
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (CLOSE_KERNEL, CLOSE_KERNEL))
    occ2 = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, kernel_close)
    if OPEN_KERNEL > 1:
        kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (OPEN_KERNEL, OPEN_KERNEL))
        occ2 = cv2.morphologyEx(occ2, cv2.MORPH_OPEN, kernel_open)
    kernel_dilate = cv2.getStructuringElement(cv2.MORPH_RECT, DILATE_KERNEL)
    return cv2.dilate(occ2, kernel_dilate)


def _draw_grid(img: np.ndarray, m_per_major: float = 0.5) -> None:
    h, w = img.shape[:2]
    x_min, x_max = WORLD_X
    y_min, y_max = WORLD_Y
    x_res = (x_max - x_min) / float(GRID_W)
    y_res = (y_max - y_min) / float(GRID_H)

    def world_to_col(x: float) -> int:
        return int(round((x - x_min) / x_res))

    def world_to_row(y: float) -> int:
        return int(round((y_max - y) / y_res))

    minor = (210, 210, 210)
    major = (185, 185, 185)
    start_x = np.ceil(x_min / m_per_major) * m_per_major
    x = start_x
    while x <= x_max + 1e-6:
        col = world_to_col(x)
        if 0 <= col < w:
            cv2.line(img, (col, 0), (col, h - 1), major, 1)
        x += m_per_major

    start_y = np.ceil(y_min / m_per_major) * m_per_major
    y = start_y
    while y <= y_max + 1e-6:
        row = world_to_row(y)
        if 0 <= row < h:
            cv2.line(img, (0, row), (w - 1, row), major, 1)
        y += m_per_major


def main():
    pipeline, profile, align, depth_scale = auto_start_realsense()
    print("Streaming safe occupancy. Press 'q' in window to exit.")

    robot_col = int(round((0.0 - WORLD_X[0]) / CELL_M))
    robot_row = int(round((WORLD_Y[1] - 0.0) / CELL_M))
    sensor_rc = (robot_row, robot_col)
    x_res = (WORLD_X[1] - WORLD_X[0]) / float(GRID_W)
    y_res = (WORLD_Y[1] - WORLD_Y[0]) / float(GRID_H)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            if align is not None:
                frames = align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_np = np.asanyarray(color_frame.get_data())
            depth_np = np.asanyarray(depth_frame.get_data())
            intr = build_o3d_intrinsic_from_frame(color_frame)

            pcd_cam = frames_to_pointcloud_o3d(
                color_np,
                depth_np,
                intr,
                depth_trunc=DEPTH_TRUNC,
                voxel_size=0.0,
                nb_neighbors=0,
            )
            if pcd_cam.has_points():
                pcd_filt = pcd_cam.voxel_down_sample(voxel_size=VOXEL_SIZE)
                if pcd_filt.has_points():
                    pcd_filt, _ = pcd_filt.remove_statistical_outlier(
                        nb_neighbors=NB_NEIGHBORS,
                        std_ratio=STD_RATIO,
                    )
            else:
                pcd_filt = o3d.geometry.PointCloud()

            T_local_cam = make_T_local_cam(
                cam_tx=0.0,
                cam_ty=0.0,
                cam_z=CAM_HEIGHT,
                cam_yaw_rad=np.deg2rad(CAM_YAW_DEG),
                pitch_deg=CAM_PITCH_DEG,
            )
            pcd_base = transform_pcd(pcd_filt, T_local_cam)
            pcd_roi = _crop_roi(pcd_base)

            occ_raw = _pcd_to_occ(pcd_roi)
            occ_safe = _apply_safe_filters(occ_raw)

            occ_rgb = cv2.cvtColor(occ_safe, cv2.COLOR_GRAY2BGR)
            draw_backward_fan_reference(
                occ_rgb,
                sensor_rc,
                x_res=x_res,
                y_res=y_res,
                radius_m=cfg.BACKWARD_FAN_RADIUS_M,
                fan_deg=cfg.BACKWARD_FAN_DEG,
            )
            _draw_grid(occ_rgb)
            if 0 <= robot_row < GRID_H and 0 <= robot_col < GRID_W:
                cv2.rectangle(
                    occ_rgb,
                    (robot_col - 3, robot_row - 2),
                    (robot_col + 3, robot_row + 2),
                    (0, 255, 255),
                    -1,
                )

            display = cv2.resize(occ_rgb, (GRID_W * 8, GRID_H * 8), interpolation=cv2.INTER_NEAREST)
            cv2.imshow("Safe ROI occupancy (live)", display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break

    finally:
        cv2.destroyAllWindows()
        pipeline.stop()


if __name__ == "__main__":
    main()
