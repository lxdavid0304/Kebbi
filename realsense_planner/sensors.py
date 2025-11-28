from __future__ import annotations

import math
import time
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import open3d as o3d
import pyrealsense2 as rs

from . import config as cfg
from .occupancy import meters_to_cell_xy
from .logging_utils import (
    _dbg,
    _safe_log,
    _safe_log_exc,
    _safe_log_ok,
    log,
    log_exc,
    log_ok,
)

# Expose config constants locally for legacy helpers that reference globals().
globals().update({name: getattr(cfg, name) for name in cfg.__all__})


def check_frames_match(color_np, depth_np, intr):
    Hc, Wc = color_np.shape[:2]
    Hd, Wd = depth_np.shape[:2]
    ok = True
    print(f"[chk] color: {color_np.shape} {color_np.dtype}")
    print(f"[chk] depth: {depth_np.shape} {depth_np.dtype}")
    print(f"[chk] intrinsic: {intr.width}x{intr.height}")
    if (Hc, Wc) != (Hd, Wd):
        print(f"[chk] ❌ color 與 depth 尺寸不同: {(Hc,Wc)} vs {(Hd,Wd)}")
        ok = False
    if (intr.width, intr.height) != (Wd, Hd):
        print(f"[chk] ❌ 內參尺寸與影像不符: intr={intr.width}x{intr.height} vs img={Wd}x{Hd}")
        ok = False
    if color_np.dtype != np.uint8:
        print(f"[chk] ⚠️ color 建議是 uint8，目前是 {color_np.dtype}")
    if depth_np.dtype not in (np.uint16, np.float16, np.float32, np.float64):
        print(f"[chk] ⚠️ depth 非常規 dtype: {depth_np.dtype}")
    if np.issubdtype(depth_np.dtype, np.floating):
        bad = np.isnan(depth_np).sum() + np.isinf(depth_np).sum()
        if bad > 0:
            print(f"[chk] ⚠️ depth 浮點含有 NaN/Inf: {bad} 個")
    zeros = (depth_np == 0).sum() if depth_np.dtype != np.bool_ else 0
    total = depth_np.size
    print(f"[chk] depth 零值比例: {zeros}/{total} ({zeros/total:.1%})")
    print("[chk] ✅ 通過" if ok else "[chk] ❌ 不通過（請先修正）")
    return ok


def assert_intrinsic_and_images(color_np, depth_np, intr):
    Hc, Wc = color_np.shape[:2]
    Hd, Wd = depth_np.shape[:2]
    Hi, Wi = intr.height, intr.width
    print(f"[chk] color={color_np.shape}, dtype={color_np.dtype}")
    print(f"[chk] depth={depth_np.shape}, dtype={depth_np.dtype}")
    print(f"[chk] intrinsic={Wi}x{Hi}")
    assert Hc == Hd and Wc == Wd, \
        f"❌ color 與 depth 尺寸不一致: color={Hc}x{Wc}, depth={Hd}x{Wd}"
    assert Wd == Wi and Hd == Hi, \
        f"❌ depth 尺寸 {Wd}x{Hd} 與 intrinsic {Wi}x{Hi} 不一致"
    assert color_np.dtype == np.uint8, \
        f"❌ color dtype 應該是 uint8, 拿到 {color_np.dtype}"
    assert depth_np.dtype in (np.uint16, np.float32), \
        f"❌ depth dtype 應該是 uint16(mm) 或 float32(m), 拿到 {depth_np.dtype}"
    if np.issubdtype(depth_np.dtype, np.floating):
        bad = np.isnan(depth_np).sum() + np.isinf(depth_np).sum()
        assert bad == 0, f"❌ depth 裡面有 NaN/Inf ({bad} 個)"
    print("[chk] ✅ 全部檢查通過")


def report_depth_dtype(depth_np):
    if depth_np.dtype == np.uint16:
        print("[dtype] depth 是 uint16（多半是毫米 mm）")
    elif depth_np.dtype == np.float16:
        print("[dtype] depth 是 float16（精度較低，建議先轉 float32）")
    elif depth_np.dtype == np.float32:
        print("[dtype] depth 是 float32（通常是公尺 m）")
    elif depth_np.dtype == np.float64:
        print("[dtype] depth 是 float64（雙精度，通常不必要）")
    else:
        print(f"[dtype] depth 是 {depth_np.dtype}（非常規類型）")

    # 額外印出最小/最大值（若是浮點，順便猜單位）
    try:
        vmin = float(np.nanmin(depth_np))
        vmax = float(np.nanmax(depth_np))
        print(f"[dtype] 深度值範圍: {vmin:.6f} ~ {vmax:.6f}")
        if np.issubdtype(depth_np.dtype, np.floating):
            # 粗略猜測單位
            if vmax > 10.0: print("[dtype] 這個上限偏大，可能是毫米或未縮放的單位")
            else:           print("[dtype] 上限在幾公尺內，較像『公尺 m』")
    except Exception:
        pass


def _noop(*args, **kwargs): pass


def depth_raw_to_meters(depth_np: np.ndarray, rs_scale: float,
                        near_m: float = 0.0, far_m: float | None = None) -> np.ndarray:
    """
    RealSense Z16 -> float32(公尺)。0/NaN/Inf 視為無效→設 0。
    可選近遠裁剪：不在 [near, far] 範圍內的設 0（方便之後做投影/佔據）。
    """
    d = depth_np.astype(np.float32) * float(rs_scale)   # 轉公尺
    d[~np.isfinite(d)] = 0.0
    d[d <= 0.0] = 0.0
    if far_m is not None:
        mask = (d < near_m) | (d > far_m)
        d[mask] = 0.0
    return d


def auto_start_realsense(
    w=640, h=480, fps=30,
    warmup_frames=30,
    laser_on=True,
    align_to_depth: bool = None  # None → 讀全域 USE_ALIGNED_TO_DEPTH；True=align到depth，False=align到color
):
    """
    啟動 RealSense 並回傳 (pipeline, profile, align, rs_scale)。
    - 對齊目標：依 align_to_depth / 全域 USE_ALIGNED_TO_DEPTH 決定。
    - 會幫你暖機、設定投射器（若支援）、取 depth_scale。
    """
    step = "auto_start_realsense"
    pipeline = None
    try:
        # ---------- 準備對齊目標 ----------
        if align_to_depth is None:
            try:
                align_to_depth = bool(globals().get("USE_ALIGNED_TO_DEPTH", True))
            except Exception:
                align_to_depth = True
        align_target = rs.stream.depth if align_to_depth else rs.stream.color

        log(step, "建立 context / 查裝置")
        ctx = rs.context()
        devs = ctx.query_devices()
        log(step, f"發現裝置數={len(devs)}")
        if len(devs) == 0:
            raise RuntimeError("找不到 RealSense 裝置")

        # ---------- 建立 pipeline / config ----------
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)

        log(step, "啟動 pipeline")
        profile = pipeline.start(config)

        # ---------- 裝置資訊（可選） ----------
        dev = profile.get_device()
        try:
            name = dev.get_info(rs.camera_info.name)
            sn   = dev.get_info(rs.camera_info.serial_number)
            log(step, f"裝置：{name} / S/N: {sn}")
        except Exception:
            pass

        # ---------- 投射器設定（若支援） ----------
        try:
            depth_sensor = dev.first_depth_sensor()
            if depth_sensor.supports(rs.option.emitter_enabled):
                depth_sensor.set_option(rs.option.emitter_enabled, 1 if laser_on else 0)
                log(step, f"投射器已{'啟用' if laser_on else '關閉'}")
        except Exception:
            log(step, "投射器設定略過")

        # ---------- 對齊器 ----------
        align = rs.align(align_target)
        log(step, f"對齊目標 = {'depth' if align_to_depth else 'color'}")

        # ---------- 暖機（用 aligned frames） ----------
        log(step, f"暖機丟 {warmup_frames} 幀（含對齊）")
        for _ in range(int(warmup_frames)):
            frames = pipeline.wait_for_frames()
            _ = align.process(frames)  # 丟掉即可

        # ---------- 取得 depth_scale ----------
        depth_sensor = dev.first_depth_sensor()
        rs_scale = depth_sensor.get_depth_scale()
        log(step, f"depth_scale={rs_scale}")
        log_ok(step)

        return pipeline, profile, align, rs_scale

    except Exception:
        # 若已啟動，失敗時幫你停掉，避免占住相機
        try:
            if pipeline is not None:
                pipeline.stop()
        except Exception:
            pass
        log_exc(step)
        raise


def colorize_depth(
    depth: np.ndarray,
    *,
    auto_range: bool = True,
    max_m: float = 3.5,
    depth_scale_m: Optional[float] = None,           # RealSense 請傳 scale (m/unit)
    pct_lo_hi: Tuple[float, float] = (2.0, 98.0),
    return_gray: bool = False
) -> np.ndarray:
    """
    將深度影像轉成可視化圖（預設回傳 BGR 的 JET 色圖）。
    - uint16 視為 raw，會用 depth_scale_m 轉公尺；未提供則 0.001。
    - float 視為公尺。
    - auto_range=True 用分位數拉伸；False 則固定 0~max_m。
    """
    d = np.asarray(depth)

    # 轉公尺 float32
    if d.dtype == np.uint16:
        scale = 0.001 if depth_scale_m is None else float(depth_scale_m)
        dm = d.astype(np.float32) * scale
    else:
        dm = d.astype(np.float32)

    # 無效值歸零
    bad = ~np.isfinite(dm)
    if bad.any():
        _dbg(f"[colorize_depth] 發現非數值/無限：{bad.sum()} 個，已歸零。")
    dm[bad] = 0.0
    dm[dm <= 0.0] = 0.0

    # 對比拉伸 → 0..1
    if auto_range:
        nz = dm[dm > 0]
        if nz.size:
            lo, hi = np.percentile(nz, pct_lo_hi)
            if not np.isfinite(lo): lo = 0.0
            if (not np.isfinite(hi)) or hi <= lo:
                hi = lo + 1e-6
            vis = np.clip((dm - lo) / (hi - lo), 0.0, 1.0)
        else:
            _dbg("[colorize_depth] 影像全 0，回傳全黑。")
            vis = np.zeros_like(dm, dtype=np.float32)
    else:
        vis = np.clip(dm / float(max_m), 0.0, 1.0)

    u8 = (vis * 255.0).astype(np.uint8)
    if return_gray:
        return u8
    # 延後 import，避免無 GUI 環境時一開始就報錯
    import cv2
    return cv2.applyColorMap(u8, cv2.COLORMAP_JET)


def place_human_robot_on_6x6_center_back(
    ROI: Optional[Dict[str, float]] = None,
    GRID_SIZE: Optional[int] = None,
    *,
    debug: bool = False
) -> Tuple[Tuple[Tuple[float, float], Tuple[float, float]],
           Tuple[Optional[Tuple[int, int]], Tuple[int, int]]]:
    """
    初始化放置「盲友(開發用 fallback)」與「機器人」到 6×6 世界，並換算成格座標。
    ✦ 符合最新規則：此函式【不】做後向扇形檢查、【不】限制人↔機器人距離。
    ✦ 執行期的守護扇形與真人位置，請由後鏡頭偵測 + overlay_kernel_on_occ 處理。
    
    回傳：
        ((human_x, human_y), (robot_x, robot_y)), (human_rc_or_None, robot_rc)
    其中 human_rc 可能為 None（若 HUMAN_START 不在 ROI 內）
    """
    G = globals()
    ROI = ROI or G["ROI"]
    GRID_SIZE = GRID_SIZE or G["GRID_SIZE"]

    # --- 讀全域（世界座標；+y 向前）---
    human_xy = G.get("HUMAN_START", (0.0, -1.5))   # 只作 fallback/顯示用
    robot_xy = G.get("ROBOT_START", (0.0, -0.5))   # 機器人起點（anchor）

    # --- ROI 半開區間檢查 ---
    x_min, x_max = ROI["x_min"], ROI["x_max"]
    y_min, y_max = ROI["y_min"], ROI["y_max"]

    def inside(x: float, y: float) -> bool:
        return (x_min <= x < x_max) and (y_min <= y < y_max)

    hx, hy = human_xy
    rx, ry = robot_xy

    # 機器人必須在 ROI 內（硬條件）
    if not inside(rx, ry):
        raise ValueError(f"Robot outside ROI: robot=({rx:.2f},{ry:.2f}) "
                         f"ROI.x=[{x_min},{x_max}), ROI.y=[{y_min},{y_max})")

    # 人可以不在 ROI（fallback 只是顯示用）
    human_in_roi = inside(hx, hy)

    # --- 世界 → 格座標（半開；y 大 → row 小）---
    robot_rc = meters_to_cell_xy(rx, ry, ROI=ROI, GRID_SIZE=GRID_SIZE)
    if robot_rc is None:
        raise ValueError("Robot meters_to_cell_xy failed; check ROI/GRID_SIZE.")

    human_rc = (meters_to_cell_xy(hx, hy, ROI=ROI, GRID_SIZE=GRID_SIZE)
                if human_in_roi else None)

    # --- 心跳列印（僅 debug=True；每 PLACE6X6_LOG_RATE_SEC 秒印一次）---
    if debug:
        rate = float(G.get("PLACE6X6_LOG_RATE_SEC", 2.0))
        now  = time.monotonic()
        last = getattr(place_human_robot_on_6x6_center_back, "_last_log_t", 0.0)
        if now - last >= rate:
            cx = 0.5 * (x_min + x_max); cy = 0.5 * (y_min + y_max)
            print(f"[place6x6] center=({cx:.2f},{cy:.2f}) | "
                  f"human=({hx:.2f},{hy:.2f}) rc={human_rc} | "
                  f"robot=({rx:.2f},{ry:.2f}) rc={robot_rc}")
            place_human_robot_on_6x6_center_back._last_log_t = now

    return ((hx, hy), (rx, ry)), (human_rc, robot_rc)


def _Rz(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[ c, -s, 0],
                     [ s,  c, 0],
                     [ 0,  0, 1]], dtype=np.float32)


def make_T_local_cam(
    *,
    cam_tx: float = 0.0,        # 相機相對底盤：X（右）
    cam_ty: float = 0.0,        # 相機相對底盤：Y（前）
    cam_z:  float = 0.0,        # 相機相對底盤：Z（上）
    cam_yaw_rad: float = 0.0,   # 相機相對底盤在平面旋轉（弧度）
    pitch_deg: float = 0.0      # 相機下俯角（度）
) -> np.ndarray:
    """
    RealSense 相機座標 → 機器人 Local 座標 的 4×4 變換。
    座標慣例：
      - 相機:   Xc=右, Yc=下, Zc=前
      - Local:  X=右,  Y=前,  Z=上（你的全域）
    對齊：X=Xc, Y=Zc, Z=-Yc ，再去俯角，再加相機偏航與平移。
    """
    # 相機 → Local 的軸對齊
    R_align = np.array([[1,0,0],
                        [0,0,1],
                        [0,-1,0]], dtype=np.float32)

    # 去俯角（繞 X 軸旋 -pitch）
    th = np.deg2rad(pitch_deg)
    c,s = np.cos(-th), np.sin(-th)
    Rx = np.array([[1,0,0],
                   [0,c,-s],
                   [0,s, c]], dtype=np.float32)

    # 相機在底盤上的偏航（繞 Z）
    Rz = _Rz(cam_yaw_rad)

    R = Rz @ Rx @ R_align
    t = np.array([cam_tx, cam_ty, cam_z], dtype=np.float32)

    T = np.eye(4, dtype=np.float32)
    T[:3,:3] = R
    T[:3, 3] = t
    return T


def transform_pcd(pcd_cam: o3d.geometry.PointCloud, T_local_cam: np.ndarray) -> o3d.geometry.PointCloud:
    """
    相機點雲 → 機器人 Local 座標點雲（不依賴世界位姿）。
    """
    if pcd_cam is None or len(pcd_cam.points) == 0:
        return o3d.geometry.PointCloud()
    try:
        pcd_out = pcd_cam.clone()
    except Exception:
        pcd_out = pcd_cam.select_by_index(np.arange(len(pcd_cam.points)))
    pcd_out.transform(T_local_cam.astype(np.float64))
    return pcd_out


def transform_cam_to_base(
    points: np.ndarray,
    *,
    pitch_deg: float = 0.0,
    h_cam: float = 0.0,
    cam_tx: float = 0.0,
    cam_ty: float = 0.0,
    cam_yaw_deg: float = 0.0,
    yaw_correction_deg: float = 0.0,
) -> np.ndarray:
    """
    Transform Nx3 camera points to the robot-local base frame.
    """
    if points.size == 0:
        return np.zeros((0, 3), dtype=np.float32)
    T = make_T_local_cam(
        cam_tx=cam_tx,
        cam_ty=cam_ty,
        cam_z=h_cam,
        cam_yaw_rad=math.radians(cam_yaw_deg + yaw_correction_deg),
        pitch_deg=pitch_deg,
    )
    pts = np.asarray(points, dtype=np.float32)
    rotated = (T[:3, :3] @ pts.T).T
    translated = rotated + T[:3, 3]
    return translated


def is_depth_occluded(
    depth_np: np.ndarray,
    *,
    center_frac: float = 0.9,          # 只看中央 90%
    min_m: float = 0.05,
    max_m: float = 10.0,
    invalid_ratio_thresh: float = 0.70,
    depth_scale_m: float | None = None,  # ✅ 新增：uint16 時可用 scale 轉公尺
    return_ratio: bool = False           # ✅ 新增：需要時回傳無效比例
):
    """
    True → 這幀深度大多數無效（遮擋/失效），建議不要用來更新地圖。
    uint16: 0 視為無效；若提供 depth_scale_m，會套用 min_m/max_m 檢查。
    float:  NaN/Inf 或 ≤min_m 或 >max_m 視為無效。
    只統計中央區域。
    """
    if depth_np is None or depth_np.size == 0:
        return (True, 1.0) if return_ratio else True

    # 夾限中心比例，避免意外
    cf = float(np.clip(center_frac, 1e-3, 1.0))
    h, w = depth_np.shape[:2]
    dh = int((1.0 - cf) * h / 2.0)
    dw = int((1.0 - cf) * w / 2.0)
    roi = depth_np[dh:h - dh, dw:w - dw]

    if roi.dtype == np.uint16:
        inv = (roi == 0)
        # 若有 scale，就把 uint16 轉公尺再做上下限檢查
        if depth_scale_m is None:
            # 若全域有 DEPTH_SCALE，就幫你用
            try:
                depth_scale_m = float(globals().get("DEPTH_SCALE", None))
            except Exception:
                depth_scale_m = None
        if depth_scale_m is not None:
            dm = roi.astype(np.float32) * float(depth_scale_m)
            inv |= (~np.isfinite(dm)) | (dm <= min_m) | (dm > max_m)
    else:
        dm = roi.astype(np.float32, copy=False)
        inv = (~np.isfinite(dm)) | (dm <= min_m) | (dm > max_m)

    invalid_ratio = float(inv.sum()) / inv.size if inv.size else 1.0
    occluded = (invalid_ratio > invalid_ratio_thresh)
    return (occluded, invalid_ratio) if return_ratio else occluded


def build_o3d_intrinsic_from_frame(any_frame,
                                   expect_w: Optional[int] = None,
                                   expect_h: Optional[int] = None,
                                   *,
                                   undistort: bool = False,
                                   dist_coeffs: Optional[np.ndarray] = None) -> o3d.camera.PinholeCameraIntrinsic:
    """
    從 RealSense 的 video frame（color/depth/aligned 都可）建立 Open3D 內參。
    備註：
      - Open3D 建點雲不處理畸變，若要更準請先 undistort_rgbd(...) 再餵入。
      - expect_w/h 可選，用來 assert 解析度是否符合你的管線。
    """
    step = "build_o3d_intrinsic_from_frame"
    try:
        vs = any_frame.profile.as_video_stream_profile()
        intr = getattr(vs, "get_intrinsics", None)
        intr = intr() if callable(intr) else vs.intrinsics  # pyrealsense2.intrinsics

        if expect_w is not None and expect_h is not None:
            if (intr.width, intr.height) != (int(expect_w), int(expect_h)):
                raise ValueError(f"內參尺寸與影像不一致: intr={intr.width}x{intr.height} vs img={expect_w}x{expect_h}")

        ointr = o3d.camera.PinholeCameraIntrinsic(
            intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy
        )

        # 提醒：Open3D 不吃畸變；若要更準請先去畸變
        try:
            model = getattr(intr, "model", None)
            if model is not None and model != 0:
                if undistort and dist_coeffs is None:
                    log(step, f"⚠️ model={model} 有畸變；你開了 undistort 但未提供 dist。請提供 cv_K/dist 再去畸變。")
                else:
                    log(step, f"ℹ️ model={model}：如需更準，請先 undistort 再建 PCD。")
        except Exception:
            pass

        log(step, f"size=({intr.width},{intr.height}) fx={intr.fx:.2f} fy={intr.fy:.2f} "
                  f"pp=({intr.ppx:.2f},{intr.ppy:.2f}) undistort={undistort}")
        return ointr
    except Exception:
        log_exc(step); raise


def undistort_rgbd(color_bgr: np.ndarray, depth_m: np.ndarray,
                   K: np.ndarray, dist: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    使用 OpenCV 先把 RGB 與『公尺』深度圖去畸變（不裁邊）。
    ★重要：若你的深度已「對齊到 Color」，就用 Color 的 K/dist；
          若是「對齊到 Depth」，就用 Depth 的 K/dist。
    """
    step = "undistort_rgbd"
    try:
        h, w = color_bgr.shape[:2]
        depth_m = np.ascontiguousarray(depth_m.astype(np.float32, copy=False))  # 保證連續/float32

        newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=1.0, newImgSize=(w, h))
        map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, newK, (w, h), cv2.CV_32FC1)

        color_und = cv2.remap(color_bgr, map1, map2, interpolation=cv2.INTER_LINEAR,  borderMode=cv2.BORDER_CONSTANT)
        depth_und = cv2.remap(depth_m,  map1, map2, interpolation=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)

        log(step, "去畸變完成(不裁邊)")
        return color_und, depth_und
    except Exception:
        log_exc(step); raise


def remove_floor_and_clip(points_local: np.ndarray,
                          *,
                          floor_eps: float = 0.01,   # RANSAC 容忍（m）
                          h_min: float = 0.05,       # 從地面起算的最小高度（m）
                          h_max: float = 1.20,       # 從地面起算的最大高度（m）
                          near_m: float = None,
                          far_m: float = None,
                          use_ransac: bool = True) -> np.ndarray:
    """
    輸入為「機器人 Local 座標」點雲（X=左右, Y=前後, Z=高度）。
    1) 先做近/遠距濾波（在 XY 平面距離）
    2) RANSAC 擬合地面平面，計算『相對地面的有號高度』
       - 法線朝上（Z>0）
       - 高度於 [h_min, h_max] 之間保留
    3) 若 RANSAC 失敗，回退為簡單的 Z 夾帶（Z ∈ [h_min, h_max]）

    備註：
    - 建議：near_m、far_m 分別帶入全域 NEAR_M、FAR_M_DEFAULT
    - 這裡的高度是以擬合到的「當前地面」為 0 的相對高度，更穩。
    """
    if points_local is None or points_local.size == 0:
        return np.empty((0, 3), np.float32)

    pts = np.asarray(points_local, dtype=np.float32)

    # 1) 近/遠距濾波（以 XY 平面半徑）
    if near_m is None:
        near_m = float(globals().get("NEAR_M", 0.30))
    if far_m is None:
        far_m = float(globals().get("FAR_M_DEFAULT", 3.50))

    rng = np.hypot(pts[:, 0], pts[:, 1])
    mask_rng = (rng >= near_m) & (rng <= far_m)
    pts = pts[mask_rng]
    if pts.size == 0:
        return pts

    # 2) RANSAC 地面擬合 → 有號高度
    if use_ransac:
        try:
            pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
            plane, inliers = pcd.segment_plane(distance_threshold=float(floor_eps),
                                               ransac_n=3, num_iterations=2000)
            a, b, c, d = [float(v) for v in plane]
            n = np.array([a, b, c], dtype=np.float32)
            norm = np.linalg.norm(n) + 1e-12
            n /= norm; d /= norm
            # 讓法線朝上（Z>0），高度「往上為正」
            if n[2] < 0:
                n = -n; d = -d
            heights = (pts @ n) + d   # 有號「相對地面」高度
            keep = (heights >= float(h_min)) & (heights <= float(h_max))
            return pts[keep]
        except Exception as e:
            print(f"[remove_floor_and_clip] RANSAC 失敗，改用 Z 夾帶：{e}")

    # 3) 回退：直接用 Z 當高度帶
    z = pts[:, 2]
    keep = (z >= float(h_min)) & (z <= float(h_max))
    return pts[keep]


def frames_to_pointcloud_o3d(
    color_np: np.ndarray,
    depth_np: np.ndarray,                          # RealSense uint16(mm) 或 float*(m)
    o3d_intrinsic: o3d.camera.PinholeCameraIntrinsic,
    *,
    depth_trunc: float = None,                     # m；None 時用 FAR_M_DEFAULT
    depth_scale_m: float | None = None,            # ← 新增：RealSense 的 rs_scale（公尺/單位）
    voxel_size: float = 0.02,
    nb_neighbors: int = 20,
    std_ratio: float = 2.0,
    depth_median_ksize: int = 5,
    depth_bilateral_d: int = 5,
    use_occlusion_guard: bool = True,              # ← 新增：先看中央是否遮擋
    undistort: bool = False,
    cv_K: Optional[np.ndarray] = None,
    dist: Optional[np.ndarray] = None,
    step_name: str = "frames_to_pointcloud_o3d"
) -> o3d.geometry.PointCloud:
    """
    產出『相機座標系』的點雲。
    - 僅做：深度→公尺、近/遠裁、去畸變(可選)、平滑、降採樣、統計去噪。
    - 不做：高度(Z)篩選、ROI(x,y)裁切 —— 這些請在『轉成 Local（Z向上）』後再做，
      以符合妳的全域座標約定（X左右、Y前後、Z高度）。
    """
    step = step_name
    if depth_trunc is None:
        depth_trunc = float(globals().get("FAR_M_DEFAULT", 3.5))

    # 1) 驗證 ---------------------------------------------------------------
    sub = f"{step}.1.validate"
    try:
        assert isinstance(color_np, np.ndarray) and isinstance(depth_np, np.ndarray), "輸入必須是 numpy.ndarray"
        Hc, Wc = color_np.shape[:2]
        Hd, Wd = depth_np.shape[:2]
        assert color_np.ndim == 3 and color_np.shape[2] == 3, "color 應為 (H,W,3) BGR/RGB"
        assert depth_np.ndim == 2, "depth 應為 (H,W)"
        assert (Hc, Wc) == (Hd, Wd), "color/depth 尺寸不一致"
        assert (Wd, Hd) == (o3d_intrinsic.width, o3d_intrinsic.height), "intrinsic 尺寸不符"
        assert float(depth_trunc) > 0.0, "depth_trunc 必須 > 0"
        _safe_log_ok(sub)
    except Exception as e:
        _safe_log_exc(sub, e); raise

    # 2) 遮擋守門（可選，減少壞幀）-----------------------------------------
    if use_occlusion_guard:
        try:
            if is_depth_occluded(depth_np):
                _safe_log(step, "⚠️ 深度遮擋/有效比例過低（中央區域），返回空點雲")
                return o3d.geometry.PointCloud()
        except Exception:
            pass

    # 3) 準備公尺深度 +（可選）去畸變 + 濾波 -------------------------------
    sub = f"{step}.2.prep_depth_m"
    try:
        color_bgr = color_np

        # 3.1 深度→公尺
        if depth_np.dtype == np.uint16:
            scale = 0.001 if depth_scale_m is None else float(depth_scale_m)
            depth_m = depth_np.astype(np.float32) * scale
        else:
            depth_m = depth_np.astype(np.float32, copy=False)
        depth_m[~np.isfinite(depth_m)] = 0.0

        # 3.2 近裁剪 + 遠截斷
        NEAR = float(globals().get("NEAR_M", 0.30))
        if NEAR > 0:
            depth_m[(depth_m > 0) & (depth_m < NEAR)] = 0.0
        np.minimum(depth_m, float(depth_trunc), out=depth_m)

        # 3.3 有效深度比例過低 → 視為失效
        total = depth_m.size
        valid_ratio = float((depth_m > 0).sum()) / float(total)
        if valid_ratio < 0.05:
            _safe_log(step, f"⚠️ 有效深度比例過低 {valid_ratio:.1%}，返回空點雲")
            return o3d.geometry.PointCloud()

        # 3.4 去畸變（可選；需提供 cv_K/dist）
        if undistort:
            if cv_K is None or dist is None:
                _safe_log(step, "⚠️ undistort=True 但未提供 cv_K/dist，跳過去畸變")
            else:
                color_bgr, depth_m = undistort_rgbd(color_bgr, depth_m, cv_K, dist)

        # 3.5 濾波：中值→雙邊
        color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)

        if depth_median_ksize and depth_median_ksize >= 3 and depth_median_ksize % 2 == 1:
            dm_mm = (depth_m * 1000.0).astype(np.uint16)
            dm_mm = cv2.medianBlur(dm_mm, int(depth_median_ksize))
            depth_m = dm_mm.astype(np.float32) * 0.001

        if depth_bilateral_d and depth_bilateral_d > 0:
            depth_m = cv2.bilateralFilter(depth_m, d=int(depth_bilateral_d),
                                          sigmaColor=0.05, sigmaSpace=5)
        _safe_log_ok(sub)
    except Exception as e:
        _safe_log_exc(sub, e); raise

    # 4) 組 RGBD（公尺；Open3D 用 depth_scale=1.0）-------------------------
    sub = f"{step}.3.make_rgbd"
    try:
        color_o3d = o3d.geometry.Image(np.require(color_rgb, dtype=np.uint8,   requirements=["C", "A", "O"]))
        depth_o3d = o3d.geometry.Image(np.require(depth_m,  dtype=np.float32, requirements=["C", "A", "O"]))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d, depth_o3d,
            depth_scale=1.0,                   # 深度單位為公尺
            depth_trunc=float(depth_trunc),
            convert_rgb_to_intensity=False
        )
        _safe_log_ok(sub)
    except Exception as e:
        _safe_log_exc(sub, e); raise

    # 5) 建點雲（相機座標系）-----------------------------------------------
    sub = f"{step}.4.create_pcd"
    try:
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_intrinsic, np.eye(4))
        npts_raw = int(np.asarray(pcd.points).shape[0])
        _safe_log(sub, f"points_raw={npts_raw}")
        _safe_log_ok(sub)
    except Exception as e:
        _safe_log_exc(sub, e); raise

    # 6) 降採樣 + 統計去噪 + 失效守門 ---------------------------------------
    sub = f"{step}.5.down_and_denoise"
    try:
        if voxel_size and voxel_size > 0:
            pcd = pcd.voxel_down_sample(voxel_size=float(voxel_size))

        if nb_neighbors and std_ratio and nb_neighbors > 0 and std_ratio > 0:
            try:
                pcd, _ = pcd.remove_statistical_outlier(
                    nb_neighbors=int(nb_neighbors), std_ratio=float(std_ratio)
                )
            except Exception:
                pass

        npts = int(np.asarray(pcd.points).shape[0])
        if npts < 500:
            _safe_log(step, f"⚠️ 去噪後點數過少 ({npts})，返回空點雲")
            return o3d.geometry.PointCloud()

        _safe_log_ok(sub)
    except Exception as e:
        _safe_log_exc(sub, e); raise

    # 7) ❌ 不在此做「高度 / ROI」裁切（留到 Local Z-up 後再做） ------------
    _safe_log(step, f"FINAL points={np.asarray(pcd.points).shape[0]}")
    _safe_log_ok(step)
    return pcd
