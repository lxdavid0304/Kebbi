#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Command-line interface for the modular RealSense D435 capture pipeline.
"""

from __future__ import annotations

import argparse
import time
from typing import Optional, Tuple, List

import cv2
import numpy as np
import open3d as o3d

from . import config as cfg
from .logging_utils import log, log_exc, log_warn, debug_env
from .occupancy import (
    meters_to_cell_xy,
    raytrace_free_from_sensor,
    xy_resolution,
    pcd_to_occupancy_from_o3d_xy,
    draw_backward_fan_reference,
)
from .planner import (
    astar_opt,
    get_goal_rc,
    get_robot_pos_rc,
    set_goal_rc,
    set_robot_pos_rc,
    world_yaw_deg_to_grid_heading,
    _on_mouse_set_goal as on_mouse_set_goal,
)
from .robot_tcp import RobotTCPClient, execute_path_over_tcp
from .sensors import (
    auto_start_realsense,
    build_o3d_intrinsic_from_frame,
    colorize_depth,
    frames_to_pointcloud_o3d,
    is_depth_occluded,
    transform_cam_to_base,
)
from .world_map import GlobalWorldMap, LocalMapMetadata, UNKNOWN, OCCUPIED, FREE

try:
    from tof_maixsense.state import get_latest_person_range as _get_latest_person_range
except Exception:  # pragma: no cover - ToF module optional at runtime
    _get_latest_person_range = None


class _SimplePersonRange:
    def __init__(self, distance_m: float):
        self.distance_m = float(distance_m)
        self.ts = time.time()


_manual_person_range: Optional[_SimplePersonRange] = None


def _update_manual_person_range(distance_m: float):
    global _manual_person_range
    try:
        _manual_person_range = _SimplePersonRange(float(distance_m))
    except Exception:
        _manual_person_range = None


def _fetch_person_range(max_age: float = 1.0):
    # 1) 後向距離（TCP 發來）優先
    if _manual_person_range is not None:
        if max_age <= 0 or (time.time() - _manual_person_range.ts) <= max_age:
            return _manual_person_range
    # 2) 若有 tof_maixsense ToF 模組就用
    if _get_latest_person_range is None:
        return None
    try:
        return _get_latest_person_range(max_age=max_age)
    except Exception:
        return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RealSense D435 capture & planner")
    parser.add_argument("--tcp-ip", default=cfg.ROBOT_TCP_IP, help="Robot TCP IP")
    parser.add_argument("--tcp-port", type=int, default=cfg.ROBOT_TCP_PORT, help="Robot TCP port")
    parser.add_argument("--no-tcp", action="store_true", help="Disable TCP commands")
    parser.add_argument("--cam-pitch", type=float, default=40.0, help="Camera pitch angle (deg)")
    parser.add_argument("--cam-height", type=float, default=0.063, help="Camera height above base (m)")
    parser.add_argument("--cam-yaw-offset", type=float, default=0.0, help="Additional yaw correction (deg)")
    parser.add_argument("--map-interval", type=float, default=cfg.MAP_UPDATE_INTERVAL_SEC, help="Seconds between map updates")
    parser.add_argument("--odom-host", default="127.0.0.1", help="Odometry TCP host")
    parser.add_argument("--odom-port", type=int, default=8888, help="Odometry TCP port")
    return parser.parse_args()


def handle_robot_pos(payload) -> None:
    if payload is None:
        return
    mode, data = payload
    if mode == "rc":
        set_robot_pos_rc((int(data[0]), int(data[1])))
    elif mode == "xy":
        rc = meters_to_cell_xy(float(data[0]), float(data[1]), ROI=cfg.ROI, GRID_SIZE=cfg.GRID_SIZE)
        if rc:
            set_robot_pos_rc(rc)


def handle_tcp_event(msg: str) -> None:
    """Handle extra TCP messages (e.g., HUMAN_DIST:<meters>)."""
    if not msg:
        return
    if msg.startswith("HUMAN_DIST:"):
        try:
            dist = float(msg.split(":", 1)[1].strip())
            _update_manual_person_range(dist)
        except Exception:
            pass


def main():
    args = parse_args()
    debug_env()

    log("main", "🎬 初始化 RealSense 管線…")
    pipeline, profile, align, depth_scale = auto_start_realsense()
    log("main", "✅ 串流開始：G=設定目標、P=規劃執行、S=停止、Q=離開")

    world_map = GlobalWorldMap()

    # Robot-centric ROI: left/right +/-1.5 m, front 2.5 m, back 0.5 m (includes robot origin)
    local_roi_front = {"x_min": -1.5, "x_max": 1.5, "y_min": -0.5, "y_max": 2.5}
    # No odom: use the same ROI so the robot origin is still inside
    local_roi_bottom = {"x_min": -1.5, "x_max": 1.5, "y_min": -0.5, "y_max": 2.5}

    def _grid_size_for_roi(roi: dict) -> int:
        cols = int((roi["x_max"] - roi["x_min"]) / cfg.CELL_M)
        rows = int((roi["y_max"] - roi["y_min"]) / cfg.CELL_M)
        return max(1, max(cols, rows))

    local_grid_size = _grid_size_for_roi(local_roi_front)

    frames = pipeline.wait_for_frames()
    if align is not None:
        frames = align.process(frames)
    color_frame = frames.get_color_frame()
    if not color_frame:
        raise RuntimeError("無法取得 color frame")
    o3d_intrinsic = build_o3d_intrinsic_from_frame(color_frame)

    window = "Occupancy (live)"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 720, 720)
    cv2.setMouseCallback(window, on_mouse_set_goal)

    start_rc = meters_to_cell_xy(cfg.ROBOT_START[0], cfg.ROBOT_START[1], ROI=cfg.ROI, GRID_SIZE=cfg.GRID_SIZE)
    goal_rc = meters_to_cell_xy(cfg.A_STAR_GOAL_WORLD[0], cfg.A_STAR_GOAL_WORLD[1], ROI=cfg.ROI, GRID_SIZE=cfg.GRID_SIZE)
    if goal_rc:
        set_goal_rc(goal_rc)

    start_heading_deg = world_yaw_deg_to_grid_heading(cfg.ROBOT_INIT_YAW_DEG)

    tcp: Optional[RobotTCPClient] = None
    if not args.no_tcp:
        try:
            tcp = RobotTCPClient(
                args.tcp_ip,
                args.tcp_port,
                on_pos=handle_robot_pos,
                on_event=handle_tcp_event,
            )
            tcp.connect()
        except Exception as exc:
            log_warn("tcp", f"無法連線至 TCP：{exc}（離線模式）")
            tcp = None

    last_map_ts = 0.0
    last_img = np.full((cfg.GRID_SIZE, cfg.GRID_SIZE, 3), 255, np.uint8)
    current_path_rc: Optional[List[Tuple[int, int]]] = None
    pending_path_to_execute: Optional[List[Tuple[int, int]]] = None
    pending_execute_ready_at: Optional[float] = None
    last_x_res = None
    last_y_res = None

    try:
        while True:
            frames = pipeline.wait_for_frames()
            if align is not None:
                frames = align.process(frames)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            color_np = np.asanyarray(color_frame.get_data())
            depth_raw = np.asanyarray(depth_frame.get_data())

            if is_depth_occluded(depth_raw):
                cv2.imshow(window, last_img)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            now = time.monotonic()
            if (now - last_map_ts) >= args.map_interval:
                last_map_ts = now

                try:
                    cv2.imshow("Color", color_np)
                    cv2.imshow("Depth", colorize_depth(depth_raw, depth_scale_m=depth_scale))
                except Exception:
                    pass

                pcd_cam = frames_to_pointcloud_o3d(
                    color_np,
                    depth_raw,
                    o3d_intrinsic,
                    depth_trunc=cfg.FAR_M_DEFAULT,
                    voxel_size=0.02,
                    nb_neighbors=20,
                    std_ratio=2.0,
                    depth_median_ksize=5,
                    depth_bilateral_d=5,
                    undistort=False,
                )

                pts_cam = np.asarray(pcd_cam.points)
                if pts_cam.size == 0:
                    cv2.imshow(window, last_img)
                    continue

                pts_base = transform_cam_to_base(
                    pts_cam,
                    pitch_deg=args.cam_pitch,
                    h_cam=args.cam_height,
                    yaw_correction_deg=args.cam_yaw_offset,
                )
                pts_xyh = np.stack([pts_base[:, 0], pts_base[:, 2], pts_base[:, 1]], axis=1)
                pcd_local = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts_xyh))

                active_roi = local_roi_front if (tcp and tcp.has_odometry_data()) else local_roi_bottom

                grid_raw, grid_block, occ_vis, extras = pcd_to_occupancy_from_o3d_xy(
                    pcd_local,
                    active_roi,
                    local_grid_size,
                    draw_annot=cfg.DRAW_GRID_ANNOTATION,
                    robot_radius_m=cfg.ROBOT_RADIUS_M,
                    min_points_per_cell=1,
                    inflate_pixels=cfg.INFLATE_CELLS,
                    open_close_kernel=cfg.OPEN_CLOSE_KERNEL,
                    sensor_world_xy=cfg.ROBOT_START,
                    use_raytrace_gating=False,
                    max_ray_range_m=cfg.FAR_M_DEFAULT,
                )

                sensor_rc = get_robot_pos_rc() or start_rc
                person_range = _fetch_person_range(max_age=1.0)
                if cfg.APPLY_VIS_FREE_TO_PLANNING and sensor_rc is not None:
                    hits = np.argwhere(grid_raw > 0)
                    free_seen = raytrace_free_from_sensor(
                        grid_block,
                        sensor_rc,
                        hits,
                        ROI=cfg.ROI,
                        GRID_SIZE=cfg.GRID_SIZE,
                        max_range_m=cfg.FAR_M_DEFAULT,
                    )
                    grid_free_mask = ((grid_block == 0) & (free_seen > 0))
                else:
                    grid_free_mask = (grid_block == 0)

                last_x_res, last_y_res = xy_resolution(cfg.ROI, cfg.GRID_SIZE)

                local_states = np.full_like(grid_block, UNKNOWN, dtype=np.uint8)
                local_states[grid_block > 0] = OCCUPIED
                local_states[grid_free_mask] = FREE
                
                # 從 TCP 客戶端獲取位姿（已合併里程計功能）
                if tcp and tcp.has_odometry_data():
                    odom_pose = tcp.get_pose()
                    # 里程計座標系轉換到世界座標系：
                    # 里程計：前進=+X, 右=+Y, heading=0為+X方向
                    # 世界：前進=+Y, 右=+X, heading=0為+X方向, heading=90為+Y方向
                    # 轉換：world_x = odom_y, world_y = odom_x, world_heading = odom_heading + 90
                    from .robot_tcp import RobotPose
                    pose = RobotPose(
                        x_m=odom_pose.y_m,
                        y_m=odom_pose.x_m,
                        heading_deg=odom_pose.heading_deg + 90.0
                    )
                else:
                    # 使用預設位姿
                    from .robot_tcp import RobotPose
                    pose = RobotPose(
                        x_m=cfg.ROBOT_START[0],
                        y_m=cfg.ROBOT_START[1],
                        heading_deg=cfg.ROBOT_INIT_YAW_DEG
                    )
                meta = LocalMapMetadata(
                    roi_x_min=active_roi["x_min"],
                    roi_x_max=active_roi["x_max"],
                    roi_y_min=active_roi["y_min"],
                    roi_y_max=active_roi["y_max"],
                    cell_m=cfg.CELL_M,
                )
                world_map.integrate_local(local_states, meta, pose)

                img = occ_vis.copy()
                # 每 0.5 m 畫一條網格線（CELL_M=0.05 => 每 10 格），方便判讀局部位置
                local_major = max(1, int(0.5 / cfg.CELL_M))
                for c in range(0, img.shape[1], local_major):
                    cv2.line(img, (c, 0), (c, img.shape[0] - 1), (180, 180, 180), 1)
                for r in range(0, img.shape[0], local_major):
                    cv2.line(img, (0, r), (img.shape[1] - 1, r), (180, 180, 180), 1)
                if current_path_rc:
                    for r, c in current_path_rc:
                        cv2.circle(img, (int(c), int(r)), 1, (0, 165, 255), -1)
                if sensor_rc is not None:
                    cv2.circle(img, (int(sensor_rc[1]), int(sensor_rc[0])), 4, (0, 255, 255), -1)
                goal = get_goal_rc()
                if goal is not None:
                    cv2.circle(img, (int(goal[1]), int(goal[0])), 4, (0, 165, 255), -1)

                if (
                    cfg.BACKWARD_FAN_ALWAYS_RENDER
                    and sensor_rc is not None
                    and last_x_res
                    and last_y_res
                ):
                    img = draw_backward_fan_reference(
                        img,
                        sensor_rc,
                        robot_yaw_deg=0.0,  # Local view: robot always faces forward (+Y)
                        x_res=last_x_res,
                        y_res=last_y_res,
                        radius_m=cfg.BACKWARD_FAN_RADIUS_M,
                        fan_deg=cfg.BACKWARD_FAN_DEG,
                        color=(255, 0, 0),
                        alpha=0.25,
                    )
                if (
                    person_range
                    and sensor_rc is not None
                    and last_x_res
                    and last_y_res
                ):
                    radius = float(
                        np.clip(person_range.distance_m, 0.15, cfg.BACKWARD_FAN_RADIUS_M)
                    )
                    img = draw_backward_fan_reference(
                        img,
                        sensor_rc,
                        robot_yaw_deg=0.0,  # Local view: robot always faces forward (+Y)
                        x_res=last_x_res,
                        y_res=last_y_res,
                        radius_m=radius,
                        fan_deg=cfg.BACKWARD_FAN_DEG,
                        color=(0, 255, 255),
                        alpha=0.6,
                    )
                # 0.5 m grid (CELL_M=0.05 -> every 10 cells), easier to read local position
                local_major = max(1, int(0.5 / cfg.CELL_M))
                for c in range(0, img.shape[1], local_major):
                    cv2.line(img, (c, 0), (c, img.shape[0] - 1), (180, 180, 180), 1)
                for r in range(0, img.shape[0], local_major):
                    cv2.line(img, (0, r), (img.shape[1] - 1, r), (180, 180, 180), 1)
                last_img = img

                world_display = np.zeros((world_map.grid_size, world_map.grid_size, 3), dtype=np.uint8)
                world_display[world_map.grid == UNKNOWN] = (40, 40, 40)
                world_display[world_map.grid == FREE] = (255, 255, 255)
                world_display[world_map.grid == OCCUPIED] = (0, 0, 0)
                if current_path_rc:
                    for r, c in current_path_rc:
                        cv2.circle(world_display, (int(c), int(r)), 1, (0, 165, 255), -1)
                if sensor_rc is not None:
                    cv2.circle(world_display, (int(sensor_rc[1]), int(sensor_rc[0])), 2, (0, 255, 255), -1)
                if (
                    cfg.BACKWARD_FAN_ALWAYS_RENDER
                    and sensor_rc is not None
                    and last_x_res
                    and last_y_res
                ):
                    # draw fan in world coordinates so it aligns with the global map
                    # pose already contains world coordinates after the transformation above
                    sensor_world_xy = (pose.x_m, pose.y_m) if pose is not None else None
                    robot_yaw = pose.heading_deg if pose is not None else 0.0
                    world_display = draw_backward_fan_reference(
                        world_display,
                        sensor_world_xy,
                        robot_yaw_deg=robot_yaw,
                        x_res=last_x_res,
                        y_res=last_y_res,
                        radius_m=cfg.BACKWARD_FAN_RADIUS_M,
                        fan_deg=cfg.BACKWARD_FAN_DEG,
                        color=(255, 0, 0),
                        alpha=0.15,
                        roi_x_min=cfg.ROI["x_min"],
                        roi_y_max=cfg.ROI["y_max"],
                    )
                if (
                    person_range
                    and sensor_rc is not None
                    and last_x_res
                    and last_y_res
                ):
                    radius = float(
                        np.clip(person_range.distance_m, 0.15, cfg.BACKWARD_FAN_RADIUS_M)
                    )
                    # pose already contains world coordinates after the transformation above
                    sensor_world_xy = (pose.x_m, pose.y_m) if pose is not None else None
                    robot_yaw = pose.heading_deg if pose is not None else 0.0
                    world_display = draw_backward_fan_reference(
                        world_display,
                        sensor_world_xy,
                        robot_yaw_deg=robot_yaw,
                        x_res=last_x_res,
                        y_res=last_y_res,
                        radius_m=radius,
                        fan_deg=cfg.BACKWARD_FAN_DEG,
                        color=(0, 255, 255),
                        alpha=0.55,
                        roi_x_min=cfg.ROI["x_min"],
                        roi_y_max=cfg.ROI["y_max"],
                    )
                    label = f"BACK dist {person_range.distance_m:.2f} m"
                    cv2.putText(
                        world_display,
                        label,
                        (10, world_display.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 0, 255),
                        1,
                        cv2.LINE_AA,
                    )
                # 0.5 m grid + enlarged world display
                major_step = max(1, int(0.5 / cfg.CELL_M))
                for c in range(0, world_map.grid_size, major_step):
                    cv2.line(world_display, (c, 0), (c, world_map.grid_size - 1), (80, 80, 80), 1)
                for r in range(0, world_map.grid_size, major_step):
                    cv2.line(world_display, (0, r), (world_map.grid_size - 1, r), (80, 80, 80), 1)
                scale = 8  # enlarge 8x for readability
                world_display_large = cv2.resize(
                    world_display,
                    (world_map.grid_size * scale, world_map.grid_size * scale),
                    interpolation=cv2.INTER_NEAREST,
                )
                cv2.imshow("World Map", world_display_large)

            cv2.imshow(window, last_img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("p"):
                try:
                    print("\n請輸入目標世界座標 (x y) 公尺：", end="", flush=True)
                    line = input().strip()
                    parts = [p for p in line.replace(",", " ").split() if p]
                    if len(parts) != 2:
                        log_warn("goal", "格式錯誤，需提供 x y")
                        continue
                    x_m, y_m = float(parts[0]), float(parts[1])
                    rc = meters_to_cell_xy(x_m, y_m, ROI=cfg.ROI, GRID_SIZE=cfg.GRID_SIZE)
                    if rc is None:
                        log_warn("goal", "座標不在 ROI 內")
                        continue
                    set_goal_rc(rc)
                    log("goal", f"設定目標 ({x_m:.2f}, {y_m:.2f}) → rc={rc}")
                except Exception as exc:
                    log_exc("goal", exc)
                    continue
                cur_rc = get_robot_pos_rc() or start_rc
                goal_rc = get_goal_rc()
                if cur_rc is None or goal_rc is None:
                    log_warn("plan", "缺少地圖或起迄點")
                    continue
                planner_grid = world_map.to_planner_grid()
                path = astar_opt(
                    planner_grid,
                    cur_rc,
                    goal_rc,
                    allow_diagonal=True,
                    diag_cost=1.41421356237,
                    avoid_corner_cut=True,
                )
                if not path:
                    log_warn("plan", "找不到路徑")
                    continue

                current_path_rc = path.copy()
                pending_path_to_execute = None
                pending_execute_ready_at = None
                if tcp and tcp.connected():
                    pending_path_to_execute = path.copy()
                    pending_execute_ready_at = time.monotonic() + 2.0
                    log("plan", "路徑已繪製，2 秒後開始導航")
                else:
                    log("plan", "未連線 TCP，只顯示路徑")
            elif key == ord("s"):
                try:
                    if tcp:
                        tcp.send_line("stop")
                except Exception:
                    pass

            if (
                tcp
                and tcp.connected()
                and pending_path_to_execute
                and pending_execute_ready_at is not None
                and time.monotonic() >= pending_execute_ready_at
            ):
                path_to_run = pending_path_to_execute
                pending_path_to_execute = None
                pending_execute_ready_at = None
                log("plan", "開始沿路徑導航")
                ok = execute_path_over_tcp(
                    path_to_run,
                    tcp,
                    start_heading_deg=float(start_heading_deg),
                    allow_diagonal=True,
                )
                log("plan", "路徑執行完成" if ok else "路徑執行失敗")
                if ok:
                    current_path_rc = None

    except KeyboardInterrupt:
        log("main", "收到 Ctrl+C，結束程式")
    finally:
        try:
            pipeline.stop()
        except Exception:
            pass
        if tcp:
            try:
                tcp.close()
            except Exception:
                pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()