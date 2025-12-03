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
from typing import Any, Dict, List, Optional
from threading import Thread

import _bootstrap  # noqa: F401
import cv2
import numpy as np
import open3d as o3d
from web import create_app

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
from tof_maixsense import state

# ROI (robot frame)
ROI_X = (-1.5, 1.5)
ROI_Y = (0.0, 3.0)
Z_RANGE = (0.2, 0.5)
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

# Voice command handling
VOICE_EVENT_STALE_SEC = 3.0  # drop stale events to keep responses snappy

# Simple straight-line avoidance parameters
FORWARD_LINE_LENGTH_M = 1.5
FORWARD_LINE_STEP_M = CELL_M
AVOID_SCAN_OFFSET_DEG = 20.0
AVOID_TURN_STEP_DEG = 20.0
AVOID_CMD_INTERVAL_SEC = 1.5
ENABLE_SIMPLE_AVOIDANCE = True

_last_voice_command: Optional[str] = None  # track to avoid repeating the same advice
_last_line_turn_dir = 1
_last_avoid_cmd_ts = 0.0


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
    print("x,y,yaw_deg:",x,y,yaw_deg)
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
    print("[dbg]roi_center_world_x, roi_center_world_y:",roi_center_world_x,roi_center_world_y)
    # rotate ROI
    # 關鍵：OpenCV 圖像座標系 Y 軸向下，世界座標系 Y 軸向上
    # 因此旋轉角度需要取負值才能正確對齊
    # 使用 borderValue=255 避免旋轉後的黑色邊角被當作障礙物
    print("[dbg] yaw_deg:", yaw_deg)
    M = cv2.getRotationMatrix2D((occ_roi.shape[1] / 2, occ_roi.shape[0] / 2), yaw_deg, 1.0)
    rotated = cv2.warpAffine(occ_roi, M, (occ_roi.shape[1], occ_roi.shape[0]),
                            flags=cv2.INTER_NEAREST, borderValue=255)
    
    # center in world
    cx = (roi_center_world_x - WORLD_X[0]) / CELL_M
    cy = (WORLD_Y[1] - roi_center_world_y) / CELL_M
    print("[dbg] cx, cy:",cx,cy)
    offset_col = int(round(cx - occ_roi.shape[1] / 2))
    offset_row = int(round(cy - occ_roi.shape[0] / 2))
    print("[dbg] offset_row, offset_col:",offset_row,offset_col)
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

def _draw_blind_person(
    world_rgb: np.ndarray,
    pose_xy_theta: tuple[float, float, float],
    detection: Optional[Dict[str, Any]],
) -> None:
    """在世界地圖上畫出盲人的估計位置。"""

    if not detection:
        return
    distance_m = detection.get("distance")
    angle_deg = detection.get("angle")
    if distance_m is None or angle_deg is None:
        return

    robot_x, robot_y, robot_yaw_deg = pose_xy_theta

    # 世界座標系：robot_yaw_deg (0°=右/+X, 90°=前/+Y)
    # 後方基準方向 = robot_yaw - 180°
    robot_yaw_rad = math.radians(robot_yaw_deg)
    base_dir_rad = robot_yaw_rad - math.pi
    # 盲人相對角度（從後方基準往左為正）
    phi_rad = base_dir_rad - math.radians(angle_deg) + math.pi / 2

    # 計算盲人在世界座標系中的位置
    human_x = robot_x + distance_m * math.cos(phi_rad)
    human_y = robot_y + distance_m * math.sin(phi_rad)

    # 轉換為圖像座標
    col = int(round((human_x - WORLD_X[0]) / CELL_M))
    row = int(round((WORLD_Y[1] - human_y) / CELL_M))

    if 0 <= row < GRID_H and 0 <= col < GRID_W:
        cv2.circle(world_rgb, (col, row), 3, (0, 0, 255), -1)


def _sample_occupancy_line(
    world_img: np.ndarray,
    pose_xy_theta: tuple[float, float, float],
    yaw_deg: float,
    length_m: float = FORWARD_LINE_LENGTH_M,
    step_m: float = FORWARD_LINE_STEP_M,
) -> Dict[str, Any]:
    """Sample cells along a ray and report whether an obstacle blocks it."""
    yaw_rad = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    points: List[tuple[int, int]] = []
    blocked = False
    hit_distance = length_m
    max_steps = max(1, int(round(length_m / max(step_m, 1e-3))))
    for step_idx in range(1, max_steps + 1):
        dist = step_idx * step_m
        px = pose_xy_theta[0] + cos_yaw * dist
        py = pose_xy_theta[1] + sin_yaw * dist
        col = int(round((px - WORLD_X[0]) / CELL_M))
        row = int(round((WORLD_Y[1] - py) / CELL_M))
        if row < 0 or row >= GRID_H or col < 0 or col >= GRID_W:
            break
        points.append((row, col))
        if world_img[row, col] == 0:
            blocked = True
            hit_distance = dist
            break
    traversed = len(points) * step_m
    return {
        "points": points,
        "blocked": blocked,
        "hit_distance": hit_distance,
        "traversed": traversed,
    }


def _draw_line_segments(world_rgb: np.ndarray, points: List[tuple[int, int]], color: tuple[int, int, int]) -> None:
    if not points:
        return
    if len(points) == 1:
        row, col = points[0]
        world_rgb[row, col] = color
        return
    pts = np.array([(col, row) for row, col in points], dtype=np.int32)
    cv2.polylines(world_rgb, [pts], False, color, 1, cv2.LINE_AA)


def _plan_straight_line_avoidance(
    world_img: np.ndarray,
    pose_xy_theta: tuple[float, float, float],
) -> Dict[str, Any]:
    """Return avoidance info by probing straight ahead and slight left/right."""
    global _last_line_turn_dir

    info: Dict[str, Any] = {}
    forward = _sample_occupancy_line(world_img, pose_xy_theta, pose_xy_theta[2])
    info["forward"] = forward
    if not forward["blocked"]:
        info["blocked"] = False
        info["turn_dir"] = 0
        info["turn_deg"] = None
        return info

    left = _sample_occupancy_line(world_img, pose_xy_theta, pose_xy_theta[2] + AVOID_SCAN_OFFSET_DEG)
    right = _sample_occupancy_line(world_img, pose_xy_theta, pose_xy_theta[2] - AVOID_SCAN_OFFSET_DEG)

    def _score(line: Dict[str, Any]) -> float:
        if not line["points"]:
            return 0.0
        if line["blocked"]:
            return line["traversed"]
        return FORWARD_LINE_LENGTH_M

    left_score = _score(left)
    right_score = _score(right)

    direction = 0
    if left_score > right_score:
        direction = 1
    elif right_score > left_score:
        direction = -1
    else:
        direction = _last_line_turn_dir or 1

    _last_line_turn_dir = direction or _last_line_turn_dir or 1
    info.update(
        {
            "blocked": True,
            "turn_dir": direction,
            "turn_deg": direction * AVOID_TURN_STEP_DEG if direction else None,
            "left": left,
            "right": right,
        }
    )
    return info


def _maybe_send_avoidance_command(
    tcp: Optional[RobotTCPClient],
    avoidance_info: Dict[str, Any],
) -> None:
    """Send a small turn command if the forward line is blocked."""
    global _last_avoid_cmd_ts

    if (
        not ENABLE_SIMPLE_AVOIDANCE
        or not avoidance_info.get("blocked")
        or not tcp
        or not tcp.connected()
    ):
        return

    turn_deg = avoidance_info.get("turn_deg")
    if not turn_deg:
        return

    now = time.time()
    if now - _last_avoid_cmd_ts < AVOID_CMD_INTERVAL_SEC:
        return

    try:
        cmd_value = int(round(turn_deg))
        tcp.send_line(f"turn:{cmd_value}")
        _last_avoid_cmd_ts = now
        print(f"[avoid] forward blocked, send turn:{cmd_value}")
    except Exception as exc:
        print(f"[avoid] failed to send command: {exc}")

def _select_detection_entry(snapshot: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    meta = snapshot.get("meta") or {}
    detections: List[Dict[str, Any]] = snapshot.get("detections") or []
    last_detection_flag = meta.get("last_detection")
    if detections and last_detection_flag != "none":
        return detections[-1]
    last_seen = meta.get("last_seen_pose")
    if isinstance(last_seen, dict):
        return {
            "distance": last_seen.get("distance"),
            "angle": last_seen.get("angle"),
            "status": meta.get("last_detection_status", "unknown"),
            "tts_text": None,
            "timestamp": last_seen.get("timestamp", 0.0),
        }
    if detections:
        return detections[-1]
    return None


def _deliver_voice_events(tcp: Optional[RobotTCPClient], voice_events: List[Dict[str, Any]]) -> None:
    global _last_voice_command

    if not tcp or not tcp.connected() or not voice_events:
        return

    now = time.time()
    for event in sorted(voice_events, key=lambda e: e.get("event_id", 0)):
        command = event.get("command")
        event_id = event.get("event_id")
        if not command or event_id is None:
            continue

        event_ts = float(event.get("timestamp") or now)
        if VOICE_EVENT_STALE_SEC and (now - event_ts) > VOICE_EVENT_STALE_SEC:
            state.mark_voice_processed(event_id)
            print(f"[voice] 丟棄過期語音事件 {event_id}: {command}")
            continue

        if _last_voice_command == command:
            state.mark_voice_processed(event_id)
            print(f"[voice] 忽略重複語音事件 {event_id}: {command}")
            continue

        try:
            tcp.send_line(f"tts:{command}")
            state.mark_voice_processed(event_id)
            _last_voice_command = command
            print(f"[voice] 已轉送事件 {event_id}: {command}")
        except Exception as exc:
            print(f"[voice] 傳送語音事件 {event_id} 失敗：{exc}")
            break


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

# 全局變量用於在 Flask 中共享當前顯示畫面
display = None

def main():
    global display
    
    # 啟動 Flask web server 在背景線程
    app = create_app()
    flask_thread = Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False))
    flask_thread.daemon = True
    flask_thread.start()
    print("[web] Flask server 已啟動於 http://0.0.0.0:5000")
    
    # 初始化單一 TCP 客戶端（包含命令和里程計功能）
    tcp = _init_tcp()
    app.tcp_client = tcp  # 將 TCP 客戶端傳遞給 Flask app
    last_tcp_attempt = time.time()
    tcp_status_cache: Optional[str] = None

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
            if tcp is None and (time.time() - last_tcp_attempt) > 5.0:
                tcp = _init_tcp()
                app.tcp_client = tcp  # 更新 Flask app 的 TCP 客戶端引用
                last_tcp_attempt = time.time()

            snapshot: Dict[str, Any] = {"detections": [], "voice_events": [], "meta": {}}
            try:
                snapshot = state.read_pending_events()
            except Exception as exc:
                print(f"[state] 讀取共享狀態失敗：{exc}")
            detection_entry = _select_detection_entry(snapshot)
            voice_events = snapshot.get("voice_events", [])
            meta = snapshot.get("meta", {})

            if voice_events:
                _deliver_voice_events(tcp, voice_events)

            if tcp is None:
                tcp_status_value = "not_ready"
            elif tcp.connected():
                tcp_status_value = "connected"
            else:
                tcp_status_value = "connecting"
            if tcp_status_value != tcp_status_cache:
                try:
                    state.update_meta(tcp_status=tcp_status_value)
                except Exception as exc:
                    print(f"[state] 無法更新 TCP 狀態：{exc}")
                tcp_status_cache = tcp_status_value

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
                # POSM 目前回報的前進方向在 X 軸上為負值，因此這裡額外取負號，讓世界 +Y 仍代表實際前進。
                # 里程計：前進=+X(實際回報為負), 右=+Y, heading=0為+X方向
                # 世界：前進=+Y, 右=+X, heading=0為+X方向, heading=90為+Y方向
                # 轉換：world_x = odom_y, world_y = -odom_x, world_heading = odom_heading + 90
                world_x = od_pose.y_m
                world_y = -od_pose.x_m
                world_yaw = od_pose.heading_deg + 90.0
                pose = (world_x, world_y, world_yaw)
            else:
                print(f"[odom] no data yet, using default pose: {pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.1f}")

            try:
                state.update_odom(
                    position={"x": pose[0], "y": pose[1]},
                    orientation={"yaw": pose[2]},
                )
            except Exception as exc:
                print(f"[state] 無法寫入里程計：{exc}")

            world = np.full((GRID_H, GRID_W), 255, np.uint8)
            _integrate_roi_into_world(occ_roi, pose, world)

            world_rgb = cv2.cvtColor(world, cv2.COLOR_GRAY2BGR)
            sensor_rc = (
                int(round((WORLD_Y[1] - pose[1]) / CELL_M)),
                int(round((pose[0] - WORLD_X[0]) / CELL_M)),
            )

            avoidance_info = _plan_straight_line_avoidance(world, pose)
            forward_line = avoidance_info.get("forward", {})
            forward_pts = forward_line.get("points") or []
            forward_color = (0, 255, 0) if not avoidance_info.get("blocked") else (0, 0, 255)
            _draw_line_segments(world_rgb, forward_pts, forward_color)
            if avoidance_info.get("blocked"):
                _draw_line_segments(
                    world_rgb,
                    (avoidance_info.get("left") or {}).get("points") or [],
                    (0, 165, 255),
                )
                _draw_line_segments(
                    world_rgb,
                    (avoidance_info.get("right") or {}).get("points") or [],
                    (255, 165, 0),
                )
            _maybe_send_avoidance_command(tcp, avoidance_info)

            # 扇形區域需要使用相反的旋轉方向以正確跟隨機器人後方
            draw_backward_fan_reference(
                world_rgb,
                sensor_rc,
                robot_yaw_deg=-pose[2],
                x_res=CELL_M,
                y_res=CELL_M,
                radius_m=cfg.BACKWARD_FAN_RADIUS_M,
                fan_deg=cfg.BACKWARD_FAN_DEG,
            )

            # ⭐ 使用 state.py 的資料，在世界地圖上標示盲人位置
            _draw_blind_person(world_rgb, pose, detection_entry)

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
            display_local = cv2.resize(world_rgb, (GRID_W * 8, GRID_H * 8), interpolation=cv2.INTER_NEAREST)
            
            status_text = []

            if tcp and tcp.connected():
                status_text.append(("TCP: connected", (0, 255, 0)))
            elif tcp:
                status_text.append(("TCP: connecting...", (0, 165, 255)))
            else:
                status_text.append(("TCP: not ready", (128, 128, 128)))

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

            status_text.append((f"position: ({pose[0]:.2f}, {pose[1]:.2f}) angle: {pose[2]:.0f}°", (100, 100, 100)))

            det_status = meta.get("last_detection_status", "unknown") if isinstance(meta, dict) else "unknown"
            if detection_entry and detection_entry.get("status"):
                det_status = detection_entry.get("status")
            det_color = (0, 255, 0) if det_status in ("NORMAL", "none") else (0, 0, 255)
            status_text.append((f"detection: {det_status}", det_color))
            if avoidance_info.get("blocked"):
                dir_label = "left" if avoidance_info.get("turn_dir", 1) >= 0 else "right"
                status_text.append((f"avoid: turn {dir_label}", (0, 0, 255)))
            else:
                status_text.append(("avoid: clear", (0, 255, 0)))
            voice_color = (0, 255, 0) if not voice_events else (0, 165, 255)
            status_text.append((f"voice queue: {len(voice_events)}", voice_color))
            
            # 在畫面左上角顯示狀態
            y_offset = 20
            for text, color in status_text:
                cv2.putText(display_local, text, (10, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color,
                           1, cv2.LINE_AA)
                y_offset += 20
            
            # 顯示按鍵說明
            cv2.putText(display_local, "W/S/A/D: 前後左右 | X: 停止 | Q: 結束",
                       (10, display_local.shape[0] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
            
            # 更新全局 display 變數供 Flask 使用
            app.display = display_local.copy()
            
            #cv2.imshow("Dynamic world occupancy (raw)", display_local)
            #key = cv2.waitKey(1) & 0xFF
            key = -1
            if key == ord("q") or key == 27:
                break
            if tcp and tcp.connected() and key in (ord("w"), ord("s"), ord("a"), ord("d"), ord("x"), ord("c")):
                try:
                    if key == ord("w"):
                        tcp.send_line("move:0.5")
                        print("[cmd] 發送: forward")
                    elif key == ord("s"):
                        tcp.send_line("move:-0.5")
                        print("[cmd] 發送: backward")
                    elif key == ord("a"):
                        tcp.send_line("turn:45")
                        print("[cmd] 發送: turn:45")
                    elif key == ord("d"):
                        tcp.send_line("turn:-45")
                        print("[cmd] 發送: turn:-45")
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
