#!/usr/bin/env python3
"""
Single-frame test: capture D435, build point cloud, apply light filtering,
transform to robot base frame (same math as capture_d435), crop 3x3 ROI, and
render a small occupancy map with 0.5 m grid lines (cell size 5 cm).

ROI (robot frame):
    x in [-1.5, 1.5]  (left/right)
    y in [0.0, 3.0]   (forward, robot at (0,0))

Outputs (Desktop):
    - color PNG, depth PNG (16-bit)
    - PLY: raw_cam, filt_cam, filt_base (after transform), filt_roi (cropped)
    - Occupancy PNG with 0.5 m grid and robot marker

Requires: pyrealsense2, open3d, numpy, cv2, d435capture.sensors/config
"""

from __future__ import annotations

import time
from pathlib import Path

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


# ROI around robot (meters)
ROI_X = (-1.5, 1.5)
ROI_Y = (0.0, 3.0)
CELL_M = 0.05  # 5 cm resolution
# Local ROI grid (3m x 3m at 5cm -> 60x60)
ROI_GRID_W = int(round((ROI_X[1] - ROI_X[0]) / CELL_M))  # 60
ROI_GRID_H = int(round((ROI_Y[1] - ROI_Y[0]) / CELL_M))  # 60
# Global 6x6 m canvas (same 5 cm cell, 120x120)
WORLD_X = (-3.0, 3.0)
WORLD_Y = (-3.0, 3.0)
WORLD_GRID_W = int(round((WORLD_X[1] - WORLD_X[0]) / CELL_M))  # 120
WORLD_GRID_H = int(round((WORLD_Y[1] - WORLD_Y[0]) / CELL_M))  # 120

VOXEL_SIZE = 0.01  # light downsample (1 cm)
NB_NEIGHBORS = 30
STD_RATIO = 2.5
DEPTH_TRUNC = 3.5
Z_RANGE = (0.2, 1.0)  # tighter height band to drop floor

# Yaw correction if camera faces +Y (add 90 deg); keep 0 if already aligned
CAM_YAW_DEG = 0.0


def _desktop() -> Path:
    d = Path.home() / "Desktop"
    return d if d.exists() else Path.cwd()


def _flip_pcd(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    """Flip Y/Z for nicer viewing."""
    out = o3d.geometry.PointCloud(pcd)
    out.transform(
        [
            [1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [0, 0, 0, 1],
        ]
    )
    return out


def _pcd_crop_roi_xy(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
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


def _pcd_to_occ_roi(pcd: o3d.geometry.PointCloud) -> np.ndarray:
    """Top-down projection to a 3x3m ROI grid (1=free, 0=occupied)."""
    occ = np.full((ROI_GRID_H, ROI_GRID_W), 255, np.uint8)
    if not pcd.has_points():
        return occ
    pts = np.asarray(pcd.points)
    xs, ys, zs = pts[:, 0], pts[:, 1], pts[:, 2]
    mask = (zs >= Z_RANGE[0]) & (zs <= Z_RANGE[1])
    xs, ys = xs[mask], ys[mask]
    cols = ((xs - ROI_X[0]) / CELL_M).astype(int)
    rows = ((ROI_Y[1] - ys) / CELL_M).astype(int)
    valid = (rows >= 0) & (rows < ROI_GRID_H) & (cols >= 0) & (cols < ROI_GRID_W)
    rows, cols = rows[valid], cols[valid]
    occ[rows, cols] = 0
    return occ


def _pcd_to_occ_world(pcd: o3d.geometry.PointCloud) -> np.ndarray:
    """Project cropped ROI points onto 6x6 world grid."""
    occ = np.full((WORLD_GRID_H, WORLD_GRID_W), 255, np.uint8)
    if not pcd.has_points():
        return occ
    pts = np.asarray(pcd.points)
    xs, ys, zs = pts[:, 0], pts[:, 1], pts[:, 2]
    mask = (zs >= Z_RANGE[0]) & (zs <= Z_RANGE[1])
    xs, ys = xs[mask], ys[mask]
    cols = ((xs - WORLD_X[0]) / CELL_M).astype(int)
    rows = ((WORLD_Y[1] - ys) / CELL_M).astype(int)
    valid = (rows >= 0) & (rows < WORLD_GRID_H) & (cols >= 0) & (cols < WORLD_GRID_W)
    rows, cols = rows[valid], cols[valid]
    occ[rows, cols] = 0
    return occ


def _draw_grid(img: np.ndarray, x_range, y_range, grid_w: int, grid_h: int, m_per_major: float = 0.5) -> None:
    """Draw 0.5 m grid lines on an RGB img using provided extents."""
    h, w = img.shape[:2]
    x_min, x_max = x_range
    y_min, y_max = y_range
    x_res = (x_max - x_min) / float(grid_w)
    y_res = (y_max - y_min) / float(grid_h)

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
            is_major = abs((x / m_per_major) - round(x / m_per_major)) < 1e-6
            cv2.line(img, (col, 0), (col, h - 1), major if is_major else minor, 1)
        x += m_per_major

    start_y = np.ceil(y_min / m_per_major) * m_per_major
    y = start_y
    while y <= y_max + 1e-6:
        row = world_to_row(y)
        if 0 <= row < h:
            is_major = abs((y / m_per_major) - round(y / m_per_major)) < 1e-6
            cv2.line(img, (0, row), (w - 1, row), major if is_major else minor, 1)
        y += m_per_major


def main():
    pipeline, profile, align, depth_scale = auto_start_realsense()
    print("Stream started. Press Enter to capture one frame...")
    input()

    frames = pipeline.wait_for_frames()
    if align is not None:
        frames = align.process(frames)
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    if not color_frame or not depth_frame:
        raise RuntimeError("Missing color/depth frame")

    color_np = np.asanyarray(color_frame.get_data())
    depth_np = np.asanyarray(depth_frame.get_data())
    intr = build_o3d_intrinsic_from_frame(color_frame)

    # Build camera-frame point cloud (no filter in first pass)
    pcd_cam = frames_to_pointcloud_o3d(
        color_np,
        depth_np,
        intr,
        depth_trunc=DEPTH_TRUNC,
        voxel_size=0.0,
        nb_neighbors=0,
    )
    print("pcd_cam pts:", len(pcd_cam.points))

    # Light filter
    if pcd_cam.has_points():
        pcd_filt = pcd_cam.voxel_down_sample(voxel_size=VOXEL_SIZE)
        if pcd_filt.has_points():
            pcd_filt, _ = pcd_filt.remove_statistical_outlier(
                nb_neighbors=NB_NEIGHBORS,
                std_ratio=STD_RATIO,
            )
    else:
        pcd_filt = o3d.geometry.PointCloud()
    print("pcd_filt pts:", len(pcd_filt.points))

    # Camera -> robot base transform (using same math as capture_d435 defaults)
    T_local_cam = make_T_local_cam(
        cam_tx=0.0,
        cam_ty=0.0,
        cam_z=0.06,             # 6 cm above base
        cam_yaw_rad=np.deg2rad(CAM_YAW_DEG),
        pitch_deg=-20.0,        # +pitch is down, so -20 = 20 deg up tilt
    )
    pcd_base = transform_pcd(pcd_filt, T_local_cam)
    print("pcd_base pts:", len(pcd_base.points))
    if pcd_base.has_points():
        ys = np.asarray(pcd_base.points)[:, 1]
        print("base y min/max:", float(ys.min()), float(ys.max()))
        pts = np.asarray(pcd_base.points)
        xs, ys, zs = pts[:, 0], pts[:, 1], pts[:, 2]
        print("base x min/max:", float(xs.min()), float(xs.max()))
        print("base y min/max (again):", float(ys.min()), float(ys.max()))
        print("base z min/max:", float(zs.min()), float(zs.max()))
    print(f"Z_RANGE filter: {Z_RANGE[0]} .. {Z_RANGE[1]}")

    # Crop ROI
    pcd_roi = _pcd_crop_roi_xy(pcd_base)
    print("pcd_roi pts:", len(pcd_roi.points))

    # Occupancy (ROI view)
    occ_roi = _pcd_to_occ_roi(pcd_roi)
    occ_roi_rgb = cv2.cvtColor(occ_roi, cv2.COLOR_GRAY2BGR)
    _draw_grid(occ_roi_rgb, ROI_X, ROI_Y, ROI_GRID_W, ROI_GRID_H, m_per_major=0.5)
    robot_col_roi = int(round((0.0 - ROI_X[0]) / CELL_M))
    robot_row_roi = int(round((ROI_Y[1] - 0.0) / CELL_M))
    if 0 <= robot_row_roi < ROI_GRID_H and 0 <= robot_col_roi < ROI_GRID_W:
        cv2.rectangle(
            occ_roi_rgb,
            (robot_col_roi - 3, robot_row_roi - 2),
            (robot_col_roi + 3, robot_row_roi + 2),
            (0, 255, 255),
            -1,
        )
    occ_roi_big = cv2.resize(
        occ_roi_rgb,
        (ROI_GRID_W * 8, ROI_GRID_H * 8),
        interpolation=cv2.INTER_NEAREST,
    )

    # Occupancy (world 6x6) using the same 3x3 ROI points pasted into world map
    occ_world = _pcd_to_occ_world(pcd_roi)
    occ_world_rgb = cv2.cvtColor(occ_world, cv2.COLOR_GRAY2BGR)
    _draw_grid(occ_world_rgb, WORLD_X, WORLD_Y, WORLD_GRID_W, WORLD_GRID_H, m_per_major=0.5)
    robot_col_w = int(round((0.0 - WORLD_X[0]) / CELL_M))
    robot_row_w = int(round((WORLD_Y[1] - 0.0) / CELL_M))
    x_res_w = (WORLD_X[1] - WORLD_X[0]) / float(WORLD_GRID_W)
    y_res_w = (WORLD_Y[1] - WORLD_Y[0]) / float(WORLD_GRID_H)
    draw_backward_fan_reference(
        occ_world_rgb,
        (robot_row_w, robot_col_w),
        x_res=x_res_w,
        y_res=y_res_w,
        radius_m=cfg.BACKWARD_FAN_RADIUS_M,
        fan_deg=cfg.BACKWARD_FAN_DEG,
    )
    if 0 <= robot_row_w < WORLD_GRID_H and 0 <= robot_col_w < WORLD_GRID_W:
        cv2.rectangle(
            occ_world_rgb,
            (robot_col_w - 3, robot_row_w - 2),
            (robot_col_w + 3, robot_row_w + 2),
            (0, 255, 255),
            -1,
        )
    occ_world_big = cv2.resize(
        occ_world_rgb,
        (WORLD_GRID_W * 5, WORLD_GRID_H * 5),
        interpolation=cv2.INTER_NEAREST,
    )

    # Save
    out_dir = _desktop()
    ts = time.strftime("%Y%m%d_%H%M%S")
    img_path = out_dir / f"roi_occ_{ts}.png"
    world_img_path = out_dir / f"world_occ_{ts}.png"
    color_path = out_dir / f"roi_color_{ts}.png"
    depth_path = out_dir / f"roi_depth_{ts}.png"
    p_raw = out_dir / f"roi_raw_cam_{ts}.ply"
    p_filt = out_dir / f"roi_filt_cam_{ts}.ply"
    p_base = out_dir / f"roi_filt_base_{ts}.ply"
    p_roi = out_dir / f"roi_filt_roi_{ts}.ply"

    color_bgr = cv2.cvtColor(color_np, cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(color_path), color_bgr)
    cv2.imwrite(str(depth_path), depth_np)
    cv2.imwrite(str(img_path), occ_roi_big)
    cv2.imwrite(str(world_img_path), occ_world_big)
    o3d.io.write_point_cloud(str(p_raw), pcd_cam)
    o3d.io.write_point_cloud(str(p_filt), pcd_filt)
    o3d.io.write_point_cloud(str(p_base), pcd_base)
    o3d.io.write_point_cloud(str(p_roi), pcd_roi)

    print(f"Saved ROI occupancy -> {img_path}")
    print(f"Saved world occupancy -> {world_img_path}")
    print(f"Saved color     -> {color_path}")
    print(f"Saved depth     -> {depth_path}")
    print(f"Saved PLY raw_cam  -> {p_raw}")
    print(f"Saved PLY filt_cam -> {p_filt}")
    print(f"Saved PLY filt_base-> {p_base}")
    print(f"Saved PLY filt_roi -> {p_roi}")

    # Show color before point cloud windows
    cv2.imshow("Color frame", color_bgr)
    cv2.waitKey(1)

    # Show point clouds
    o3d.visualization.draw_geometries([_flip_pcd(pcd_filt)], window_name="Cam frame (filtered, flipped)")
    o3d.visualization.draw_geometries([_flip_pcd(pcd_base)], window_name="Base frame (filtered, flipped)")
    o3d.visualization.draw_geometries([_flip_pcd(pcd_roi)], window_name="ROI cropped (filtered, flipped)")
    cv2.imshow("ROI occupancy (0.5m grid)", occ_roi_big)
    cv2.imshow("World occupancy 6x6 (ROI pasted)", occ_world_big)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    pipeline.stop()


if __name__ == "__main__":
    main()
