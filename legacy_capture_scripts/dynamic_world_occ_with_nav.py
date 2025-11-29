#!/usr/bin/env python3
"""
Dynamic world occupancy + navigation overlay.

This script keeps the original RealSense → ROI → world-map flow from
`capture_d435_dynamic_world_occ.py` but augments it with the planner
tooling from `realsense_planner`:

* Every ROI slice is fused into a persistent `GlobalWorldMap`.
* The global grid feeds A*; operators can type target world coordinates
  (press `P`) to see a planned path.
* The aggregated map, robot pose, blind-person marker, and optional
  backward safety fan are all rendered alongside manual WASD/TCP control.
"""
from __future__ import annotations

import math
import sys
import time
from typing import List, Optional, Tuple

import _bootstrap  # noqa: F401
import cv2
import numpy as np
import open3d as o3d

from realsense_planner import config as cfg
from realsense_planner.robot_tcp import RobotTCPClient
from realsense_planner.occupancy import draw_backward_fan_reference, meters_to_cell_xy
from realsense_planner.sensors import (
    auto_start_realsense,
    build_o3d_intrinsic_from_frame,
    frames_to_pointcloud_o3d,
    make_T_local_cam,
    transform_pcd,
)
from realsense_planner.world_map import GlobalWorldMap, LocalMapMetadata, UNKNOWN, OCCUPIED, FREE
from realsense_planner.odometry import RobotPose
from realsense_planner.planner import (
    astar_opt,
    set_robot_pos_rc,
)

try:
    from tof_maixsense.state import get_latest_person_range
except Exception:  # pragma: no cover - ToF module optional
    get_latest_person_range = None

# ROI (robot frame)
ROI_X = (-1.5, 1.5)
ROI_Y = (0.0, 3.0)
Z_RANGE = (0.2, 1.0)
# Local ROI grid size (3m x 3m @ 5cm)
ROI_GRID_W = int(round((ROI_X[1] - ROI_X[0]) / cfg.CELL_M))
ROI_GRID_H = int(round((ROI_Y[1] - ROI_Y[0]) / cfg.CELL_M))

# World map 6x6 m
WORLD_X = (-3.0, 3.0)
WORLD_Y = (-3.0, 3.0)
CELL_M = cfg.CELL_M
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
    counts = np.zeros_like(occ, dtype=np.int32)
    np.add.at(counts, (rows, cols), 1)
    occ[counts >= cfg.OCC_MIN_PTS] = 0
    return occ


def _integrate_roi_into_world(occ_roi: np.ndarray, pose_xy_theta, world_img: np.ndarray) -> None:
    """Place ROI occ into world_img using pose (x,y,yaw) for on-screen visualization."""
    x, y, yaw_deg = pose_xy_theta
    yaw_rad = math.radians(yaw_deg)

    roi_center_local_x = 0.0
    roi_center_local_y = (ROI_Y[0] + ROI_Y[1]) / 2.0  # 1.5m

    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    roi_center_world_x = x + roi_center_local_x * cos_yaw - roi_center_local_y * sin_yaw
    roi_center_world_y = y + roi_center_local_x * sin_yaw + roi_center_local_y * cos_yaw

    M = cv2.getRotationMatrix2D((occ_roi.shape[1] / 2, occ_roi.shape[0] / 2), -yaw_deg, 1.0)
    rotated = cv2.warpAffine(occ_roi, M, (occ_roi.shape[1], occ_roi.shape[0]), flags=cv2.INTER_NEAREST)

    cx = (roi_center_world_x - WORLD_X[0]) / CELL_M
    cy = (WORLD_Y[1] - roi_center_world_y) / CELL_M
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

    major = (185, 185, 185)
    x = np.ceil(x_min / m_per_major) * m_per_major
    while x <= x_max + 1e-6:
        col = world_to_col(x)
        if 0 <= col < w:
            cv2.line(img, (col, 0), (col, h - 1), major, 1)
        x += m_per_major

    y = np.ceil(y_min / m_per_major) * m_per_major
    while y <= y_max + 1e-6:
        row = world_to_row(y)
        if 0 <= row < h:
            cv2.line(img, (0, row), (w - 1, row), major, 1)
        y += m_per_major


def _worldmap_to_bgr(grid: np.ndarray) -> np.ndarray:
    """Visual palette for the persistent GlobalWorldMap grid."""
    vis = np.empty((grid.shape[0], grid.shape[1], 3), np.uint8)
    vis[:, :] = (215, 215, 215)
    vis[grid == FREE] = (245, 245, 245)
    vis[grid == OCCUPIED] = (10, 10, 10)
    vis[grid == UNKNOWN] = (190, 190, 190)
    return vis


def _draw_blind_person(world_rgb: np.ndarray, pose_xy_theta: tuple[float, float, float]) -> None:
    if get_latest_person_range is None:
        return
    reading = get_latest_person_range(max_age=0.5)
    if reading is None:
        return

    robot_x, robot_y, robot_yaw_deg = pose_xy_theta
    d = reading.distance_m
    angle_deg = reading.angle_deg
    robot_yaw_rad = math.radians(robot_yaw_deg)
    base_dir_rad = robot_yaw_rad + math.pi
    phi_rad = base_dir_rad - math.radians(angle_deg)
    human_x = robot_x + d * math.cos(phi_rad)
    human_y = robot_y + d * math.sin(phi_rad)
    col = int(round((human_x - WORLD_X[0]) / CELL_M))
    row = int(round((WORLD_Y[1] - human_y) / CELL_M))
    if 0 <= row < GRID_H and 0 <= col < GRID_W:
        cv2.circle(world_rgb, (col, row), 3, (0, 0, 255), -1)


def _init_tcp() -> RobotTCPClient | None:
    try:
        tcp = RobotTCPClient(cfg.ROBOT_TCP_IP, cfg.ROBOT_TCP_PORT)
        tcp.connect()
        return tcp
    except Exception as exc:
        print(f"[tcp] connect failed: {exc}")
        return None


def _plan_path(
    planner_grid: np.ndarray,
    start_rc: Optional[Tuple[int, int]],
    goal_rc: Optional[Tuple[int, int]],
    *,
    allow_diagonal: bool = True,
) -> List[Tuple[int, int]]:
    if start_rc is None or goal_rc is None:
        return []
    path = astar_opt(planner_grid, start_rc, goal_rc, allow_diagonal=allow_diagonal)
    return path or []


def main():
    tcp = _init_tcp()
    world_map = GlobalWorldMap(cell_m=CELL_M)
    goal_rc: Optional[Tuple[int, int]] = None
    planned_path: List[Tuple[int, int]] = []
    last_plan_ts = 0.0

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

            pose = (cfg.ROBOT_START[0], cfg.ROBOT_START[1], cfg.ROBOT_INIT_YAW_DEG)
            if tcp and tcp.has_odometry_data():
                od_pose = tcp.get_pose()
                world_x = od_pose.y_m
                world_y = od_pose.x_m
                world_yaw = od_pose.heading_deg + 90.0
                pose = (world_x, world_y, world_yaw)

            world_fast = np.full((GRID_H, GRID_W), 255, np.uint8)
            _integrate_roi_into_world(occ_roi, pose, world_fast)

            # Update persistent world map
            local_states = np.full_like(occ_roi, UNKNOWN, dtype=np.uint8)
            local_states[occ_roi == 0] = OCCUPIED
            local_states[occ_roi == 255] = FREE
            meta = LocalMapMetadata(
                roi_x_min=ROI_X[0],
                roi_x_max=ROI_X[1],
                roi_y_min=ROI_Y[0],
                roi_y_max=ROI_Y[1],
                cell_m=CELL_M,
            )
            robot_pose = RobotPose(x_m=float(pose[0]), y_m=float(pose[1]), heading_deg=float(pose[2]))
            world_map.integrate_local(local_states, meta, robot_pose)

            world_rgb = _worldmap_to_bgr(world_map.grid)
            world_rgb[world_fast == 0] = (30, 30, 30)

            draw_backward_fan_reference(
                world_rgb,
                (pose[0], pose[1]),
                robot_yaw_deg=pose[2],
                x_res=CELL_M,
                y_res=CELL_M,
                radius_m=cfg.BACKWARD_FAN_RADIUS_M,
                fan_deg=cfg.BACKWARD_FAN_DEG,
                roi_x_min=WORLD_X[0],
                roi_y_max=WORLD_Y[1],
            )

            _draw_blind_person(world_rgb, pose)

            sensor_rc = (
                int(round((WORLD_Y[1] - pose[1]) / CELL_M)),
                int(round((pose[0] - WORLD_X[0]) / CELL_M)),
            )
            if 0 <= sensor_rc[0] < GRID_H and 0 <= sensor_rc[1] < GRID_W:
                cv2.rectangle(
                    world_rgb,
                    (sensor_rc[1] - 3, sensor_rc[0] - 2),
                    (sensor_rc[1] + 3, sensor_rc[0] + 2),
                    (0, 255, 255),
                    -1,
                )

            current_rc = meters_to_cell_xy(pose[0], pose[1], ROI=cfg.ROI, GRID_SIZE=cfg.GRID_SIZE)
            set_robot_pos_rc(current_rc)

            if goal_rc and current_rc:
                now = time.time()
                if now - last_plan_ts > 0.2:
                    planner_grid = world_map.to_planner_grid()
                    planned_path = _plan_path(planner_grid, current_rc, goal_rc)
                    last_plan_ts = now

                for r, c in planned_path:
                    if 0 <= r < GRID_H and 0 <= c < GRID_W:
                        world_rgb[r, c] = (0, 165, 255)
                gr, gc = goal_rc
                if 0 <= gr < GRID_H and 0 <= gc < GRID_W:
                    cv2.circle(world_rgb, (gc, gr), 4, (0, 255, 0), -1)

            _draw_grid(world_rgb)
            display = cv2.resize(world_rgb, (GRID_W * 8, GRID_H * 8), interpolation=cv2.INTER_NEAREST)

            status_lines = []
            if tcp and tcp.connected():
                status_lines.append(("TCP: connected", (0, 255, 0)))
            elif tcp:
                status_lines.append(("TCP: connecting...", (0, 165, 255)))
            else:
                status_lines.append(("TCP: not ready", (120, 120, 120)))
            status_lines.append((f"Pose: ({pose[0]:.2f}, {pose[1]:.2f}) heading {pose[2]:.0f}°", (90, 90, 90)))
            if goal_rc:
                gx, gy = goal_rc
                status_lines.append((f"Goal rc: ({gx}, {gy})", (0, 200, 0)))
                status_lines.append((f"Path len: {len(planned_path)}", (0, 200, 200)))

            y_off = 20
            for text, color in status_lines:
                cv2.putText(display, text, (10, y_off), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
                y_off += 20

            cv2.putText(
                display,
                "W/S/A/D/X/C: robot cmds | P: set goal | O: clear goal | Q: quit",
                (10, display.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (200, 200, 200),
                1,
                cv2.LINE_AA,
            )

            cv2.imshow("Dynamic world occupancy (with nav)", display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break

            if key == ord("o"):
                goal_rc = None
                planned_path = []
                print("[nav] 已清除導航目標")

            if key == ord("p"):
                try:
                    print("\n請輸入目標世界座標 (x y) 公尺：", end="", flush=True)
                    line = sys.stdin.readline().strip()
                    parts = [p for p in line.replace(",", " ").split() if p]
                    if len(parts) != 2:
                        print("格式錯誤，需提供 x y")
                    else:
                        x_m, y_m = float(parts[0]), float(parts[1])
                        rc = meters_to_cell_xy(x_m, y_m, ROI=cfg.ROI, GRID_SIZE=cfg.GRID_SIZE)
                        if rc is None:
                            print("座標不在 6x6 世界範圍")
                        else:
                            goal_rc = rc
                            planned_path = []
                            print(f"[nav] 目標設定 ({x_m:.2f}, {y_m:.2f}) -> rc={rc}")
                except Exception as exc:
                    print(f"設定目標失敗: {exc}")

            if tcp and tcp.connected() and key in (ord("w"), ord("s"), ord("a"), ord("d"), ord("x"), ord("c")):
                try:
                    if key == ord("w"):
                        tcp.send_line("forward")
                        print("[cmd] 發送: forward")
                    elif key == ord("s"):
                        tcp.send_line("backward")
                        print("[cmd] 發送: backward")
                    elif key == ord("a"):
                        tcp.send_line("turn_left")
                        print("[cmd] 發送: turn_left")
                    elif key == ord("d"):
                        tcp.send_line("turn_right")
                        print("[cmd] 發送: turn_right")
                    elif key == ord("x"):
                        tcp.send_line("stop")
                        print("[cmd] 發送: stop")
                    elif key == ord("c"):
                        tcp.send_line("spin")
                        print("[cmd] 發送: spin")
                except Exception as exc:
                    print(f"[cmd] 發送失敗: {exc}")
            elif key in (ord("w"), ord("s"), ord("a"), ord("d"), ord("x"), ord("c")):
                if not tcp:
                    print("[cmd] ⚠️  TCP 未初始化")
                elif not tcp.connected():
                    print("[cmd] ⚠️  TCP 未連線，請確認機器人 IP 和 Port 是否正確")

    finally:
        cv2.destroyAllWindows()
        try:
            pipeline.stop()
        except Exception:
            pass
        if tcp:
            try:
                tcp.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
