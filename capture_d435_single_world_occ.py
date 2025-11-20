#!/usr/bin/env python3
"""Single-frame capture: build raw ROI occupancy, apply optional odom pose,
and render in a 6x6 world map with 28x22 cm robot rectangle plus backward fan.
"""
from __future__ import annotations

import math
import time
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d

from d435capture import config as cfg
from d435capture.occupancy import draw_backward_fan_reference
from d435capture.robot_tcp import RobotTCPClient
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

VOXEL_SIZE = 0.01
NB_NEIGHBORS = 30
STD_RATIO = 2.5
DEPTH_TRUNC = 3.5

CAM_YAW_DEG = 0.0
CAM_HEIGHT = 0.06
CAM_PITCH_DEG = -20.0


def _desktop() -> Path:
    desktop = Path.home() / "Desktop"
    return desktop if desktop.exists() else Path.cwd()


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
    cols = ((xs - ROI_X[0]) / CELL_M).astype(int)
    rows = ((ROI_Y[1] - ys) / CELL_M).astype(int)
    valid = (rows >= 0) & (rows < GRID_H) & (cols >= 0) & (cols < GRID_W)
    rows, cols = rows[valid], cols[valid]
    counts = np.zeros_like(occ, dtype=np.int32)
    np.add.at(counts, (rows, cols), 1)
    occ[counts >= cfg.OCC_MIN_PTS] = 0
    return occ


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


def _integrate_roi_into_world(occ_roi: np.ndarray, pose_xy_theta, world_img: np.ndarray) -> None:
    x, y, yaw_deg = pose_xy_theta
    yaw_rad = math.radians(yaw_deg)

    roi_m = np.zeros((occ_roi.shape[0], occ_roi.shape[1], 3), np.uint8)
    roi_m[...] = occ_roi[..., None]

    center_world = (
        (x - WORLD_X[0]) / CELL_M,
        (WORLD_Y[1] - y) / CELL_M,
    )

    scale = CELL_M / CELL_M

    M = cv2.getRotationMatrix2D((occ_roi.shape[1] / 2, occ_roi.shape[0] / 2), -yaw_deg, scale)
    rotated = cv2.warpAffine(occ_roi, M, (occ_roi.shape[1], occ_roi.shape[0]), flags=cv2.INTER_NEAREST)

    offset_col = int(round(center_world[0] - occ_roi.shape[1] / 2))
    offset_row = int(round(center_world[1] - occ_roi.shape[0] / 2))

    for r in range(rotated.shape[0]):
        for c in range(rotated.shape[1]):
            if rotated[r, c] == 0:
                world_r = r + offset_row
                world_c = c + offset_col
                if 0 <= world_r < GRID_H and 0 <= world_c < GRID_W:
                    world_img[world_r, world_c] = 0


def _get_odom_pose(tcp: RobotTCPClient) -> tuple[float, float, float] | None:
    payload = tcp.get_pose()
    if payload is None:
        return None
    mode, data = payload
    if mode == "xy":
        return float(data[0]), float(data[1]), float(data[2])
    if mode == "rc":
        r, c, heading = data
        x = WORLD_X[0] + c * CELL_M
        y = WORLD_Y[1] - r * CELL_M
        return x, y, float(heading)
    return None


def main():
    tcp = RobotTCPClient(
        host=cfg.ROBOT_TCP_IP,
        port=cfg.ROBOT_TCP_PORT,
        on_pos=None,
        on_event=None,
    )
    pose_xytheta = None
    try:
        tcp.connect()
        print("Waiting for POSM...")
        t0 = time.time()
        while time.time() - t0 < 5.0 and pose_xytheta is None:
            pose_xytheta = _get_odom_pose(tcp)
            time.sleep(0.1)
    except Exception as exc:
        print(f"[tcp] connect failed: {exc}")

    pipeline, profile, align, depth_scale = auto_start_realsense()
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
        cam_yaw_rad=math.radians(CAM_YAW_DEG),
        pitch_deg=CAM_PITCH_DEG,
    )
    pcd_base = transform_pcd(pcd_filt, T_local_cam)
    pcd_roi = _crop_roi(pcd_base)
    occ_roi = _pcd_to_occ(pcd_roi)

    world = np.full((GRID_H, GRID_W), 255, np.uint8)
    if pose_xytheta is None:
        pose_xytheta = (cfg.ROBOT_START[0], cfg.ROBOT_START[1], cfg.ROBOT_INIT_YAW_DEG)
        print("using default pose", pose_xytheta)
    _integrate_roi_into_world(occ_roi, pose_xytheta, world)

    world_rgb = cv2.cvtColor(world, cv2.COLOR_GRAY2BGR)
    sensor_rc = (
        int(round((WORLD_Y[1] - pose_xytheta[1]) / CELL_M)),
        int(round((pose_xytheta[0] - WORLD_X[0]) / CELL_M)),
    )
    draw_backward_fan_reference(
        world_rgb,
        sensor_rc,
        x_res=CELL_M,
        y_res=CELL_M,
        radius_m=cfg.BACKWARD_FAN_RADIUS_M,
        fan_deg=cfg.BACKWARD_FAN_DEG,
    )
    if 0 <= sensor_rc[0] < GRID_H and 0 <= sensor_rc[1] < GRID_W:
        cv2.rectangle(
            world_rgb,
            (sensor_rc[1] - 3, sensor_rc[0] - 2),
            (sensor_rc[1] + 3, sensor_rc[0] + 2),
            (0, 255, 255),
            -1,
        )

    _draw_grid(world_rgb)
    display = cv2.resize(world_rgb, (GRID_W * 8, GRID_H * 8), interpolation=cv2.INTER_NEAREST)
    cv2.imshow("Single-frame world occupancy", display)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    pipeline.stop()
    tcp.stop()


if __name__ == "__main__":
    main()
