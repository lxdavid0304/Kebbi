#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Modularized RealSense D435 capture pipeline that builds an occupancy grid,
plans paths with A*, and optionally streams commands to the robot over TCP.
"""

from __future__ import annotations

import argparse
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import open3d as o3d

from d435capture import config as cfg
from d435capture.logging_utils import log, log_exc, log_warn, debug_env
from d435capture.occupancy import (
    meters_to_cell_xy,
    raytrace_free_from_sensor,
    xy_resolution,
    pcd_to_occupancy_from_o3d_xy,
    draw_backward_fan_reference,
)
from d435capture.planner import (
    astar_opt,
    get_goal_rc,
    get_robot_pos_rc,
    set_goal_rc,
    set_robot_pos_rc,
    world_yaw_deg_to_grid_heading,
    _on_mouse_set_goal as on_mouse_set_goal,
)
from d435capture.robot_tcp import RobotTCPClient, execute_path_over_tcp
from d435capture.sensors import (
    auto_start_realsense,
    build_o3d_intrinsic_from_frame,
    colorize_depth,
    frames_to_pointcloud_o3d,
    is_depth_occluded,
    transform_cam_to_base,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RealSense D435 capture & planner")
    parser.add_argument("--tcp-ip", default=cfg.ROBOT_TCP_IP, help="Robot TCP IP")
    parser.add_argument("--tcp-port", type=int, default=cfg.ROBOT_TCP_PORT, help="Robot TCP port")
    parser.add_argument("--no-tcp", action="store_true", help="Disable TCP commands")
    parser.add_argument("--cam-pitch", type=float, default=40.0, help="Camera pitch angle (deg)")
    parser.add_argument("--cam-height", type=float, default=0.063, help="Camera height above base (m)")
    parser.add_argument("--map-interval", type=float, default=cfg.MAP_UPDATE_INTERVAL_SEC, help="Seconds between map updates")
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


def main():
    args = parse_args()
    debug_env()

    log("main", "🎬 初始化 RealSense 管線…")
    pipeline, profile, align, depth_scale = auto_start_realsense()
    log("main", "✅ 串流開始：G=設定目標、P=規劃執行、S=停止、Q=離開")

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
            tcp = RobotTCPClient(args.tcp_ip, args.tcp_port, on_pos=handle_robot_pos)
            tcp.connect()
        except Exception as exc:
            log_warn("tcp", f"無法連線至 TCP：{exc}（離線模式）")
            tcp = None

    last_map_ts = 0.0
    last_img = np.full((cfg.GRID_SIZE, cfg.GRID_SIZE, 3), 255, np.uint8)
    last_grid_block = None
    last_grid_free = None
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
                    ROI=None,
                    voxel_size=0.02,
                    nb_neighbors=20,
                    std_ratio=2.0,
                    depth_median_ksize=5,
                    depth_bilateral_d=5,
                    y_keep=None,
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
                )
                pts_xyh = np.stack([pts_base[:, 0], pts_base[:, 2], pts_base[:, 1]], axis=1)
                pcd_local = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts_xyh))

                grid_raw, grid_block, occ_vis, extras = pcd_to_occupancy_from_o3d_xy(
                    pcd_local,
                    cfg.ROI,
                    cfg.GRID_SIZE,
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
                    grid_free = ((grid_block == 0) & (free_seen > 0)).astype(np.uint8)
                else:
                    grid_free = (grid_block == 0).astype(np.uint8)

                last_grid_block = grid_block
                last_grid_free = grid_free
                last_x_res, last_y_res = xy_resolution(cfg.ROI, cfg.GRID_SIZE)

                img = occ_vis.copy()
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
                        x_res=last_x_res,
                        y_res=last_y_res,
                        radius_m=cfg.BACKWARD_FAN_RADIUS_M,
                        fan_deg=cfg.BACKWARD_FAN_DEG,
                    )
                last_img = img

            cv2.imshow(window, last_img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("g"):
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
            elif key == ord("p"):
                cur_rc = get_robot_pos_rc() or start_rc
                goal_rc = get_goal_rc()
                if cur_rc is None or goal_rc is None or last_grid_free is None or last_grid_block is None:
                    log_warn("plan", "缺少地圖或起迄點")
                    continue

                path = astar_opt(
                    last_grid_free,
                    cur_rc,
                    goal_rc,
                    allow_diagonal=True,
                    diag_cost=1.41421356237,
                    avoid_corner_cut=True,
                )
                if not path:
                    log_warn("plan", "找不到路徑")
                    continue

                preview = last_img.copy()
                for r, c in path:
                    cv2.circle(preview, (int(c), int(r)), 1, (0, 0, 255), -1)
                cv2.imshow(window, preview)

                if tcp and tcp.connected():
                    ok = execute_path_over_tcp(
                        path,
                        tcp,
                        start_heading_deg=float(start_heading_deg),
                        allow_diagonal=True,
                    )
                    log("plan", "路徑送出完成" if ok else "路徑執行失敗")
                else:
                    log("plan", "未連線 TCP，僅顯示路徑")
            elif key == ord("s"):
                try:
                    if tcp:
                        tcp.send_line("stop")
                except Exception:
                    pass

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
