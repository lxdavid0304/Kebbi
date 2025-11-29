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

import _bootstrap  # noqa: F401
import cv2
import numpy as np
import open3d as o3d

from realsense_planner import config as cfg
from realsense_planner.robot_tcp import RobotTCPClient
from realsense_planner.occupancy import draw_backward_fan_reference
from realsense_planner.sensors import (
    auto_start_realsense,
    build_o3d_intrinsic_from_frame,
    frames_to_pointcloud_o3d,
    make_T_local_cam,
    transform_pcd,
)
from tof_maixsense.state import get_latest_person_range

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
    # Local ROI occupancy grid (3x3m)
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
    """Place ROI occ into world_img using pose (x,y,yaw)."""
    x, y, yaw_deg = pose_xy_theta
    yaw_rad = math.radians(yaw_deg)
    
    # ROI 中心在機器人坐標系中的位置 (0, 1.5)
    # 因為 ROI_Y = (0.0, 3.0)，中心在 Y=1.5
    roi_center_local_x = 0.0
    roi_center_local_y = (ROI_Y[0] + ROI_Y[1]) / 2.0  # 1.5m
    
    # 將 ROI 中心旋轉到世界坐標系
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    roi_center_world_x = x + roi_center_local_x * cos_yaw - roi_center_local_y * sin_yaw
    roi_center_world_y = y + roi_center_local_x * sin_yaw + roi_center_local_y * cos_yaw
    
    # rotate ROI
    M = cv2.getRotationMatrix2D((occ_roi.shape[1] / 2, occ_roi.shape[0] / 2), -yaw_deg, 1.0)
    rotated = cv2.warpAffine(occ_roi, M, (occ_roi.shape[1], occ_roi.shape[0]), flags=cv2.INTER_NEAREST)
    
    # center in world
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

def _draw_blind_person(world_rgb: np.ndarray, pose_xy_theta: tuple[float, float, float]) -> None:
    """
    從 state.py 讀取最新盲人距離＋角度，
    並在世界地圖上、機器人後方畫出代表盲人的紅色質點。
    """
    reading = get_latest_person_range(max_age=0.5)  # 只接受 0.5 秒內的新資料
    if reading is None:
        return  # 沒有可靠資料就不畫

    # 機器人在世界座標中的位置與朝向
    robot_x, robot_y, robot_yaw_deg = pose_xy_theta

    d = reading.distance_m       # 水平距離（公尺）
    angle_deg = reading.angle_deg  # 右正左負，相對機器人

    # ---- 從「距離＋角度」轉成世界座標 ----
    # 假設：
    #   - robot_yaw_deg 是機器人「面向前方」的角度（世界座標下）
    #   - 盲人是「在機器人後方」，angle_deg=0 表示正後方
    #   - angle_deg > 0 表示偏向機器人右後方，angle_deg < 0 表示偏左後方
    #
    # 先算出「正後方」方向 = robot_yaw + 180°
    robot_yaw_rad = math.radians(robot_yaw_deg)
    base_dir_rad = robot_yaw_rad + math.pi  # 後方方向

    # angle_deg 右正左負，我們讓正值往右偏，就是從 base_dir_rad 順時針旋轉，因此要減
    phi_rad = base_dir_rad - math.radians(angle_deg)

    # 盲人在世界座標中的位置
    human_x = robot_x + d * math.cos(phi_rad)
    human_y = robot_y + d * math.sin(phi_rad)

    # ---- 世界座標 → grid(row, col) ----
    col = int(round((human_x - WORLD_X[0]) / CELL_M))
    row = int(round((WORLD_Y[1] - human_y) / CELL_M))

    if 0 <= row < GRID_H and 0 <= col < GRID_W:
        # 畫一個紅色小圓點代表盲人
        cv2.circle(world_rgb, (col, row), 3, (0, 0, 255), -1)


def _init_tcp() -> RobotTCPClient | None:
    """初始化單一 TCP 客戶端（包含命令和里程計功能）"""
    try:
        tcp = RobotTCPClient(cfg.ROBOT_TCP_IP, cfg.ROBOT_TCP_PORT)
        tcp.connect()
        return tcp
    except Exception as exc:
        print(f"[tcp] connect failed: {exc}")
        return None
def detect_human_position():
    try:
        f=open('/tmp/human_position.txt','r')
        lines=f.readlines()
        f.close()
        if len(lines)>=2:
            line=lines[0].strip()
            parts=line.split(',')
            if len(parts)==3:
                distance=float(parts[0])
                is_too_left=parts[1]=='1'
                is_too_right=parts[2]=='1'
                if distance>cfg.HUMAN_DETECTION_MAX_DIST_MM:
                    print(f"[human] too far {distance}")
                else:
                    if is_too_left:
                        print("[human] too left")
                    elif is_too_right:
                        print("[human] too right")
                    else:
                        print("[human] position good")
    except Exception as exc:
        print(f"[human] detect position failed: {exc}")

def main():
    # 初始化單一 TCP 客戶端（包含命令和里程計功能）
    tcp = _init_tcp()

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
                print(f"[odom] pose: x={od_pose.x_m:.2f} y={od_pose.y_m:.2f} heading={od_pose.heading_deg:.1f}")
                # 里程計座標系轉換到世界座標系：
                # 里程計：前進=+X, 右=+Y, heading=0為+X方向
                # 世界：前進=+Y, 右=+X, heading=0為+X方向, heading=90為+Y方向
                # 轉換：world_x = odom_y, world_y = odom_x, world_heading = odom_heading + 90
                world_x = od_pose.y_m
                world_y = od_pose.x_m
                world_yaw = od_pose.heading_deg + 90.0
                pose = (world_x, world_y, world_yaw)
            else:
                print(f"[odom] no data yet, using default pose: {pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.1f}")

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
                robot_yaw_deg=pose[2],
                x_res=CELL_M,
                y_res=CELL_M,
                radius_m=cfg.BACKWARD_FAN_RADIUS_M,
                fan_deg=cfg.BACKWARD_FAN_DEG,
            )

            # ⭐ 新增：從 state.py 讀盲人距離＋角度，畫出紅色質點
            _draw_blind_person(world_rgb, pose)

            # 畫機器人位置（黃色小方塊）
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
            
            # 顯示連線狀態
            status_text = []
            
            # TCP 連線狀態
            if tcp and tcp.connected():
                status_text.append(("TCP: connected", (0, 255, 0)))
            elif tcp:
                status_text.append(("TCP: connecting...", (0, 165, 255)))
            else:
                status_text.append(("TCP: not ready", (128, 128, 128)))
            
            # 里程計數據狀態
            if tcp:
                has_data = tcp.has_odometry_data()
                socket_connected = tcp.connected()
                
                if has_data:
                    if socket_connected:
                        status_text.append(("odom: connected", (0, 255, 0)))
                    else:
                        status_text.append(("odom: use last position", (0, 255, 255)))
                else:
                    status_text.append(("odom: use default", (128, 128, 128)))
            else:
                status_text.append(("odom: not ready", (128, 128, 128)))
            
            # 位姿資訊
            status_text.append((f"position: ({pose[0]:.2f}, {pose[1]:.2f}) angle: {pose[2]:.0f}°", (100, 100, 100)))
            
            # 在畫面左上角顯示狀態
            y_offset = 20
            for text, color in status_text:
                cv2.putText(display, text, (10, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color,
                           1, cv2.LINE_AA)
                y_offset += 20
            
            # 顯示按鍵說明
            cv2.putText(display, "W/S/A/D: 前後左右 | X: 停止 | Q: 結束",
                       (10, display.shape[0] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
            
            cv2.imshow("Dynamic world occupancy (raw)", display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
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
