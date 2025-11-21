#!/usr/bin/env python3
"""
Dynamic world occupancy: stream RealSense, build raw 3x3 ROI occupancy,
use odometry pose (POSM) to place it on a 6x6 world map, and show the result
in real time. Optional TCP command keys let you發 forward/back/turn commands
to the robot. No close/open/dilate filters are applied—this is the raw map.
"""
from __future__ import annotations

import math
import time

import cv2
import numpy as np
import open3d as o3d

from d435capture import config as cfg
from d435capture.odometry import OdometryClient
from d435capture.robot_tcp import RobotTCPClient
from d435capture.occupancy import draw_backward_fan_reference
from d435capture.sensors import (
    auto_start_realsense,
    build_o3d_intrinsic_from_frame,
    frames_to_pointcloud_o3d,
    make_T_local_cam,
    transform_pcd,
)

# ROI (robot frame)
ROI_X = (-1.5, 1.5)
ROI_Y = (0.0, 3.0)
Z_RANGE = (-2.0, 1.0)

# World map 6x6 m
WORLD_X = (-3.0, 3.0)
WORLD_Y = (-3.0, 3.0)
CELL_M = 0.05
GRID_W = int(round((WORLD_X[1] - WORLD_X[0]) / CELL_M))
GRID_H = int(round((WORLD_Y[1] - WORLD_Y[0]) / CELL_M))

# Point cloud filters (light)
VOXEL_SIZE = 0.01
NB_NEIGHBORS = 30
STD_RATIO = 2.5
DEPTH_TRUNC = 3.5

# Camera pose relative to robot
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
    cols = ((xs - ROI_X[0]) / CELL_M).astype(int)
    rows = ((ROI_Y[1] - ys) / CELL_M).astype(int)
    valid = (rows >= 0) & (rows < GRID_H) & (cols >= 0) & (cols < GRID_W)
    rows, cols = rows[valid], cols[valid]
    occ[rows, cols] = 0
    return occ


def _integrate_roi_into_world(occ_roi: np.ndarray, pose_xy_theta, world_img: np.ndarray) -> None:
    """Place ROI occ into world_img using pose (x,y,yaw)."""
    x, y, yaw_deg = pose_xy_theta
    # rotate ROI
    M = cv2.getRotationMatrix2D((occ_roi.shape[1] / 2, occ_roi.shape[0] / 2), -yaw_deg, 1.0)
    rotated = cv2.warpAffine(occ_roi, M, (occ_roi.shape[1], occ_roi.shape[0]), flags=cv2.INTER_NEAREST)
    # center in world
    cx = (x - WORLD_X[0]) / CELL_M
    cy = (WORLD_Y[1] - y) / CELL_M
    offset_col = int(round(cx - occ_roi.shape[1] / 2))
    offset_row = int(round(cy - occ_roi.shape[0] / 2))
    for r in range(rotated.shape[0]):
        rr = r + offset_row
        if rr < 0 or rr >= GRID_H:
            continue
        row_data = rotated[r]
        cc_start = offset_col
        for c, val in enumerate(row_data):
            if val == 0:
                cc = cc_start + c
                if 0 <= cc < GRID_W:
                    world_img[rr, cc] = 0


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


def _init_tcp_for_commands() -> RobotTCPClient | None:
    try:
        tcp = RobotTCPClient(cfg.ROBOT_TCP_IP, cfg.ROBOT_TCP_PORT)
        tcp.connect()
        return tcp
    except Exception as exc:
        print(f"[tcp] command client connect failed: {exc}")
        return None


def main():
    # start odometry
    odom = OdometryClient(host=cfg.ROBOT_TCP_IP, port=cfg.ROBOT_TCP_PORT)
    odom.start()

    tcp_cmd = _init_tcp_for_commands()

    pipeline, profile, align, depth_scale = auto_start_realsense()
    T_local_cam = make_T_local_cam(
        cam_tx=0.0,
        cam_ty=0.0,
        cam_z=CAM_HEIGHT,
        cam_yaw_rad=math.radians(CAM_YAW_DEG),
        pitch_deg=CAM_PITCH_DEG,
    )

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

            pcd_base = transform_pcd(pcd_filt, T_local_cam)
            pcd_roi = _crop_roi(pcd_base)
            occ_roi = _pcd_to_occ(pcd_roi)

            pose = (cfg.ROBOT_START[0], cfg.ROBOT_START[1], 0.0)
            if odom.is_connected():
                od_pose = odom.get_pose()
                pose = (od_pose.x_m, od_pose.y_m, od_pose.heading_deg)
            else:
                print("[odom] not connected, using default pose")

            world = np.full((GRID_H, GRID_W), 255, np.uint8)
            _integrate_roi_into_world(occ_roi, pose, world)

            world_rgb = cv2.cvtColor(world, cv2.COLOR_GRAY2BGR)
            sensor_rc = (
                int(round((WORLD_Y[1] - pose[1]) / CELL_M)),
                int(round((pose[0] - WORLD_X[0]) / CELL_M)),
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
            cv2.imshow("Dynamic world occupancy (raw)", display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
            if tcp_cmd and key in (ord("w"), ord("s"), ord("a"), ord("d"), ord("x"), ord("c")):
                try:
                    if key == ord("w"):
                        tcp_cmd.send_line("forward")
                    elif key == ord("s"):
                        tcp_cmd.send_line("backward")
                    elif key == ord("a"):
                        tcp_cmd.send_line("turn_left")
                    elif key == ord("d"):
                        tcp_cmd.send_line("turn_right")
                    elif key == ord("x"):
                        tcp_cmd.send_line("stop")
                    elif key == ord("c"):
                        tcp_cmd.send_line("spin")
                except Exception as exc:
                    print(f"[cmd] send failed: {exc}")

    finally:
        cv2.destroyAllWindows()
        try:
            pipeline.stop()
        except Exception:
            pass
        try:
            odom.stop()
        except Exception:
            pass
        if tcp_cmd:
            try:
                tcp_cmd.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
