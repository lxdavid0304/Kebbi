from __future__ import annotations

import math
import os
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import open3d as o3d

from . import config as cfg
from .logging_utils import _dbg, _safe_log, _safe_log_exc

# Provide legacy globals compatibility.
globals().update({name: getattr(cfg, name) for name in cfg.__all__})


def _check_roi_keys(ROI: Dict) -> bool:
    need = {"x_min", "x_max", "y_min", "y_max"}
    missing = need.difference(ROI.keys())
    if missing:
        _dbg(f"[ROI] 缺少鍵：{sorted(missing)}；目前鍵 = {sorted(ROI.keys())}")
        return False
    return True


def meters_to_cell_xy(x: float, y: float,
                      ROI: Optional[Dict] = None,
                      GRID_SIZE: Optional[int] = None) -> Optional[Tuple[int, int]]:
    """
    世界 (x,y)[m] -> 格 (row,col)；右/上半開：x<x_max, y<y_max。
    y 往前變大 → row 變小（影像座標慣例）。
    """
    ROI = ROI or globals()["ROI"]
    GRID_SIZE = GRID_SIZE or globals()["GRID_SIZE"]

    if not _check_roi_keys(ROI):
        _dbg("[meters_to_cell_xy] ROI key 不相容，請使用 y_min/y_max（不要用 z_*）。")
        return None

    # 半開區間：落在 ROI 外直接回 None
    if not (ROI["x_min"] <= x < ROI["x_max"] and ROI["y_min"] <= y < ROI["y_max"]):
        _dbg(f"[meters_to_cell_xy] (x,y)=({x:.3f},{y:.3f}) 超出 ROI "
             f"x=[{ROI['x_min']},{ROI['x_max']}), y=[{ROI['y_min']},{ROI['y_max']})")
        return None

    x_res = (ROI["x_max"] - ROI["x_min"]) / float(GRID_SIZE)
    y_res = (ROI["y_max"] - ROI["y_min"]) / float(GRID_SIZE)

    col = int((x - ROI["x_min"]) / x_res)
    row = int((ROI["y_max"] - y) / y_res)

    # 夾限（雙保險）
    col = int(np.clip(col, 0, GRID_SIZE - 1))
    row = int(np.clip(row, 0, GRID_SIZE - 1))
    return (row, col)


def cell_to_meters_xy(row: int, col: int,
                      ROI: Optional[Dict] = None,
                      GRID_SIZE: Optional[int] = None) -> Tuple[float, float]:
    """
    格 (row,col) -> 世界 (x,y)[m]；回傳「格心」座標。
    """
    ROI = ROI or globals()["ROI"]
    GRID_SIZE = GRID_SIZE or globals()["GRID_SIZE"]

    if not _check_roi_keys(ROI):
        _dbg("[cell_to_meters_xy] ROI key 不相容，請用 y_min/y_max。")
        # 給個保守回傳（0,0），避免炸掉；你也可改成 raise
        return 0.0, 0.0

    x_res = (ROI["x_max"] - ROI["x_min"]) / float(GRID_SIZE)
    y_res = (ROI["y_max"] - ROI["y_min"]) / float(GRID_SIZE)

    x = ROI["x_min"] + (float(col) + 0.5) * x_res
    y = ROI["y_max"] - (float(row) + 0.5) * y_res
    return float(x), float(y)


def meters_to_cell_xz(x: float, z: float,
                      ROI: Optional[Dict] = None,
                      GRID_SIZE: Optional[int] = None) -> Optional[Tuple[int, int]]:
    # 這裡特別印一下，幫你抓「是不是有人還在傳 z_* ROI」
    if ROI and ("z_min" in ROI or "z_max" in ROI):
        _dbg("[meters_to_cell_xz] 偵測到 z_min/z_max，請改為 y_min/y_max（本函式會把 z 當 y 使用）。")
    return meters_to_cell_xy(x, z, ROI=ROI, GRID_SIZE=GRID_SIZE)


def cell_to_meters_xz(row: int, col: int,
                      ROI: Optional[Dict] = None,
                      GRID_SIZE: Optional[int] = None) -> Tuple[float, float]:
    x, y = cell_to_meters_xy(row, col, ROI=ROI, GRID_SIZE=GRID_SIZE)
    return x, y


def _xy_res(roi: dict, grid_size: int):
    x_res = (float(roi["x_max"]) - float(roi["x_min"])) / float(grid_size)
    y_res = (float(roi["y_max"]) - float(roi["y_min"])) / float(grid_size)
    return x_res, y_res


def _bresenham_line(r0, c0, r1, c1):
    """整數格線（含起終點），適合網格射線。"""
    dr = abs(r1 - r0); sr = 1 if r0 < r1 else -1
    dc = abs(c1 - c0); sc = 1 if c0 < c1 else -1
    err = (dr - dc)
    r, c = r0, c0
    line = []
    while True:
        line.append((r, c))
        if r == r1 and c == c1: break
        e2 = 2 * err
        if e2 > -dc: err -= dc; r += sr
        if e2 <  dr: err += dr; c += sc
    return line


def raytrace_free_from_sensor(grid_block: np.ndarray,
                              sensor_rc: tuple[int,int],
                              targets_rc: np.ndarray,
                              ROI: dict,
                              GRID_SIZE: int,
                              max_range_m: float = cfg.FAR_M_DEFAULT) -> np.ndarray:
    """
    從 sensor_rc 向每個 target 射線，沿線未撞到障礙就標為 free=1。
    射程用 max_range_m 限制（以每格公尺數換算最大步數）。
    平面：X=左右, Y=前後（Z=高度不進網格）
    """
    H, W = grid_block.shape
    free = np.zeros((H, W), np.uint8)
    if sensor_rc is None or targets_rc is None or len(targets_rc) == 0:
        return free

    # 每格幾公尺（取較大者當一步的公尺尺度）
    x_res, y_res = _xy_res(ROI, GRID_SIZE)
    step_m = max(x_res, y_res)
    max_steps = max(1, int(max_range_m / step_m))
    r2_limit = max_steps * max_steps   # 以格為單位的距離平方上限

    sr, sc = sensor_rc
    for (hr, hc) in targets_rc:
        dr = hr - sr; dc = hc - sc
        if (dr * dr + dc * dc) > r2_limit:
            continue  # 超過射程就不畫

        line = _bresenham_line(sr, sc, hr, hc)
        steps = 0
        for (r, c) in line:
            if not (0 <= r < H and 0 <= c < W):
                break
            # 一旦撞到障礙就停止，不把障礙那格標成 free
            if grid_block[r, c] == 1:
                break
            free[r, c] = 1
            steps += 1
            if steps >= max_steps:
                break
    return free


def integrate_pointcloud_to_grid(pcd_world: o3d.geometry.PointCloud,
                                 grid_block: np.ndarray,
                                 ROI: Dict,
                                 GRID_SIZE: int,
                                 *,
                                 min_pts_per_cell: int = cfg.OCC_MIN_PTS,
                                 inflate_cells: int = cfg.INFLATE_CELLS) -> np.ndarray:
    """
    世界座標點雲→佔據格（只用 X/Y；忽略高度 Z）。
    備註：若後續要做 raytrace_free，呼叫時建議傳 inflate_cells=0，
          以 hits→occ_raw→raytrace_free→carve→inflate→open/close 的順序較穩。
    """
    H, W = grid_block.shape
    # 1) OpenCV 相容 dtype
    if grid_block.dtype != np.uint8:
        grid_block = grid_block.astype(np.uint8, copy=False)

    # 2) 防呆
    if pcd_world is None or len(pcd_world.points) == 0:
        return grid_block
    pts = np.asarray(pcd_world.points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] < 2:
        return grid_block

    x_min, x_max = float(ROI["x_min"]), float(ROI["x_max"])
    y_min, y_max = float(ROI["y_min"]), float(ROI["y_max"])

    xs = pts[:, 0]
    ys = pts[:, 1]  # 世界 y = 前後；Z=高度被忽略

    # 半開區間過濾
    mask = (xs >= x_min) & (xs < x_max) & (ys >= y_min) & (ys < y_max)
    if not np.any(mask):
        return grid_block
    xs = xs[mask]; ys = ys[mask]

    # 轉格索引
    x_res, y_res = _xy_res(ROI, GRID_SIZE)
    cols = ((xs - x_min) / x_res).astype(np.int32)         # 0..W-1
    rows = ((y_max - ys) / y_res).astype(np.int32)         # 0..H-1（y 越大 row 越小）

    valid = (rows >= 0) & (rows < H) & (cols >= 0) & (cols < W)
    if not np.any(valid):
        return grid_block
    rows = rows[valid]; cols = cols[valid]

    # 計數 → 二值佔據
    hit = np.zeros((H, W), np.uint16)
    np.add.at(hit, (rows, cols), 1)
    grid_block |= (hit >= int(min_pts_per_cell)).astype(np.uint8)

    # 安全膨脹（以「格」為半徑）
    if inflate_cells and inflate_cells > 0:
        k = 2 * int(inflate_cells) + 1
        K = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        grid_block = cv2.dilate(grid_block, K)

    # 形態學清理（補洞→去孤點）
    kernel_size = cfg.OPEN_CLOSE_KERNEL
    if kernel_size and kernel_size >= 3 and (kernel_size % 2 == 1):
        K2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        grid_block = cv2.morphologyEx(grid_block, cv2.MORPH_CLOSE, K2)
        grid_block = cv2.morphologyEx(grid_block, cv2.MORPH_OPEN,  K2)

    return grid_block


def overlay_kernel_on_occ(
    occ_vis: np.ndarray,
    robot_rc: Tuple[int, int],
    K: np.ndarray,
    anchor: Tuple[int, int],
    color: Tuple[int, int, int] = (255, 0, 0),   # BGR（預設藍）
    alpha: float = 1.0,                          # 1.0=純色覆蓋；<1.0=半透明
    *,
    save: bool = False,                          # 想存就 True
    save_dir: Optional[os.PathLike] = None,      # 不給就用 cfg.SNAP_DIR_DEFAULT
    filename: Optional[str] = None,              # 指定檔名；不給就用 timestamp/overwrite
    mode: str = "timestamp",                     # "timestamp" 或 "overwrite"
    min_interval_s: float = 0.0                  # 存檔節流；0 代表不節流
) -> np.ndarray:
    """
    只把二值遮罩 K（anchor=(col,row)）疊在可視化圖 occ_vis（HxWx3, uint8）上。
    - robot_rc=(row,col) 對齊 anchor，並自動做邊界裁切。
    - 只改 occ_vis，不動規劃用的 occ。
    - save=True 才會存檔；預設時間戳命名，可選覆寫模式與節流。
    """
    # ---- 基本檢查 ----
    assert occ_vis.ndim == 3 and occ_vis.shape[2] == 3, "occ_vis must be HxWx3 BGR"
    assert occ_vis.dtype == np.uint8, "occ_vis must be uint8"
    K = np.asarray(K); assert K.ndim == 2, "K must be 2D"
    H, W = occ_vis.shape[:2]
    kr, kc = K.shape

    ac, ar = anchor  # anchor=(col,row)
    assert 0 <= ac < kc and 0 <= ar < kr, "anchor out of K bounds or wrong order (col,row expected)"
    rr, rc = robot_rc
    assert 0 <= rr < H and 0 <= rc < W, "robot_rc out of image bounds"

    # ---- 對齊與裁邊 ----
    r0 = rr - ar
    c0 = rc - ac
    rr0, rr1 = max(r0, 0), min(r0 + kr, H)
    cc0, cc1 = max(c0, 0), min(c0 + kc, W)
    if rr0 >= rr1 or cc0 >= cc1:
        return occ_vis  # 完全超出畫面

    ks_r0 = max(0, -r0); ks_c0 = max(0, -c0)
    ks_r1 = ks_r0 + (rr1 - rr0)
    ks_c1 = ks_c0 + (cc1 - cc0)

    # ---- 疊色（只改顯示圖）----
    mask = (K[ks_r0:ks_r1, ks_c0:ks_c1] > 0)
    region = occ_vis[rr0:rr1, cc0:cc1]

    if alpha >= 1.0:
        region[mask] = color
    else:
        col = np.array(color, dtype=np.float32)
        region_f = region.astype(np.float32)
        region_f[mask] = alpha * col + (1.0 - alpha) * region_f[mask]
        region[:] = np.clip(region_f, 0, 255).astype(np.uint8)

    occ_vis[rr0:rr1, cc0:cc1] = region

    # ---- 需要時才存檔（避免硬碟爆）----
    if save:
        out_dir = Path(save_dir) if save_dir is not None else cfg.SNAP_DIR_DEFAULT
        out_dir.mkdir(parents=True, exist_ok=True)

        now = time.time()
        last_t = getattr(overlay_kernel_on_occ, "_last_save_t", 0.0)
        if (now - last_t) >= float(min_interval_s):
            if filename:
                out_path = out_dir / filename
            else:
                if mode == "overwrite":
                    out_path = out_dir / "occ_latest.png"
                else:
                    ts = time.strftime("%Y%m%d_%H%M%S", time.localtime(now))
                    ms = int((now - int(now)) * 1000)
                    out_path = out_dir / f"occ_{ts}_{ms:03d}.png"

            cv2.imwrite(str(out_path), occ_vis)
            overlay_kernel_on_occ._last_save_t = now

    return occ_vis


def draw_backward_fan_reference(
    img_color: np.ndarray,
    sensor_rc: Optional[Tuple[int, int]],
    *,
    x_res: float,
    y_res: float,
    radius_m: float,
    fan_deg: float,
    color: Tuple[int, int, int] = (0, 165, 255),
    alpha: float = 0.35,
    num_samples: Optional[int] = None,
) -> np.ndarray:
    """
    以機器人為中心向後的扇形參考（僅視覺提示，不改動規劃格）。
    """
    if (
        sensor_rc is None
        or img_color is None
        or img_color.size == 0
        or radius_m <= 0
        or fan_deg <= 0
    ):
        return img_color

    h, w = img_color.shape[:2]
    cy, cx = int(sensor_rc[0]), int(sensor_rc[1])
    if not (0 <= cy < h and 0 <= cx < w):
        return img_color

    samples = num_samples or cfg.SEMICIRCLE_THETAS
    samples = max(3, int(samples))
    start_deg = 180.0 - fan_deg / 2.0
    end_deg = 180.0 + fan_deg / 2.0
    angles = np.linspace(start_deg, end_deg, samples)

    pts = [(cx, cy)]
    for deg in angles:
        rad = math.radians(deg)
        vx = math.sin(rad)
        vy = math.cos(rad)
        row = cy - (vy * radius_m) / y_res
        col = cx + (vx * radius_m) / x_res
        row = int(np.clip(round(row), 0, h - 1))
        col = int(np.clip(round(col), 0, w - 1))
        pts.append((col, row))

    pts_arr = np.array(pts, dtype=np.int32).reshape((-1, 1, 2))
    overlay = img_color.copy()
    cv2.fillPoly(overlay, [pts_arr], color)
    cv2.addWeighted(overlay, alpha, img_color, 1 - alpha, 0, dst=img_color)
    return img_color


def _make_targets_from_occ(grid_occ: np.ndarray,
                           sensor_rc: tuple[int,int],
                           max_range_cells: int | None = None) -> np.ndarray:
    """
    替 raytrace_free_from_sensor 準備目標格：每一欄往前（row 變小）找最近障礙；
    若無障礙，打到上邊界或射程前緣。
    """
    H, W = grid_occ.shape
    sr, sc = sensor_rc
    start = max(sr - 1, 0)
    targets = []
    for c in range(W):
        hit_r = -1
        for r in range(start, -1, -1):  # 往前 = row 變小（符合 +Y）
            if grid_occ[r, c] == 1:
                hit_r = r; break
        if hit_r >= 0:
            targets.append((hit_r, c))  # 最近障礙
        else:
            if max_range_cells is None:
                targets.append((0, c))  # 到上邊界
            else:
                targets.append((max(sr - max_range_cells, 0), c))
    return np.asarray(targets, dtype=np.int32)


def pcd_to_occupancy_from_o3d(
    pcd: o3d.geometry.PointCloud,
    ROI: Dict,
    grid_size: int,
    *,
    draw_annot: bool = False,
    robot_radius_m: float = None,
    min_points_per_cell: int = 1,
    inflate_pixels: Optional[int] = None,
    open_close_kernel: Optional[int] = None,
    aggregate_by_nearest: bool = True,
    z_keep: Optional[Tuple[float, float]] = None,
    sensor_world_xy: Tuple[float, float] = (0.0, 0.0),
    use_raytrace_gating: bool = True,
    max_ray_range_m: float = None,
):
    """
    要求：pcd 已在『Local（X右, Y前, Z上）』座標系。

    回傳：
      grid_raw     ：命中格（二值，1=有點命中）
      grid_block   ：障礙格（二值，1=障礙；含膨脹/形態學清理/（可選）半圓疊加）
      img_color    ：視覺化（白=可行；黑=障礙）
      extras       ：dict，含
         - sensor_rc: (row,col) 或 None
         - free_geom: 幾何清理後的自由格（二值）
         - free_ray : raytrace「看過才自由」遮罩（二值；若未啟用則全 1）
         - free_plan: 規劃實際可走（= free_geom & free_ray 若啟用；否則 free_geom）
    """
    step = "pcd_to_occupancy_from_o3d"
    try:
        H = W = int(grid_size)
        x_min, x_max = float(ROI["x_min"]), float(ROI["x_max"])
        y_min, y_max = float(ROI["y_min"]), float(ROI["y_max"])
        assert x_max > x_min and y_max > y_min, "ROI 無效"

        if robot_radius_m is None:
            robot_radius_m = float(cfg.ROBOT_RADIUS_M)
        if inflate_pixels is None:
            default_inflate = getattr(cfg, "INFLATE_CELLS", None)
            if default_inflate is None:
                default_inflate = max(
                    1,
                    int(np.ceil(robot_radius_m / max((x_max - x_min) / W, (y_max - y_min) / H))),
                )
            inflate_pixels = int(default_inflate)
        if open_close_kernel is None:
            open_close_kernel = int(cfg.OPEN_CLOSE_KERNEL)
        if max_ray_range_m is None:
            max_ray_range_m = float(cfg.FAR_M_DEFAULT)

        x_res, y_res = _xy_res(ROI, grid_size)

        # 空點雲：回傳全可行（僅視覺用途）
        if pcd is None or len(pcd.points) == 0:
            grid_empty = np.zeros((H, W), np.uint8)
            img = np.dstack([grid_empty * 0 + 255] * 3)
            extras = dict(sensor_rc=None,
                          free_geom=(1 - grid_empty),
                          free_ray=np.ones_like(grid_empty, np.uint8),
                          free_plan=(1 - grid_empty))
            return grid_empty, grid_empty, img, extras

        # 取有限點 + （可選）高度帶過濾（Z）
        pts = np.asarray(pcd.points)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if z_keep is not None:
            z0, z1 = float(z_keep[0]), float(z_keep[1])
            pts = pts[(pts[:, 2] >= z0) & (pts[:, 2] <= z1)]
        if pts.size == 0:
            grid_empty = np.zeros((H, W), np.uint8)
            img = np.dstack([grid_empty * 0 + 255] * 3)
            extras = dict(sensor_rc=None,
                          free_geom=(1 - grid_empty),
                          free_ray=np.ones_like(grid_empty, np.uint8),
                          free_plan=(1 - grid_empty))
            return grid_empty, grid_empty, img, extras

        # === 點雲投影到格（X→col, Y→row；row=0 在最上 = y_max 端） ===
        gx = np.floor((pts[:, 0] - x_min) / x_res).astype(np.int32)
        gr = np.floor((y_max - pts[:, 1]) / y_res).astype(np.int32)  # ← 重要：用 Y 決定 row
        in_mask = (gx >= 0) & (gx < W) & (gr >= 0) & (gr < H)
        gx = gx[in_mask]; gr = gr[in_mask]

        if aggregate_by_nearest:
            flat = gr * W + gx
            hit = np.zeros(H * W, np.uint8)
            hit[flat] = 1
            grid_raw = hit.reshape(H, W)
        else:
            acc = np.zeros((H, W), np.uint16)
            idx = gr * W + gx
            np.add.at(acc.ravel(), idx, 1)
            grid_raw = (acc >= int(min_points_per_cell)).astype(np.uint8)

        # === 幾何小膨脹（安全距） ===
        if inflate_pixels and inflate_pixels > 0:
            k_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                (2*inflate_pixels+1, 2*inflate_pixels+1))
            grid_block = cv2.dilate(grid_raw.astype(np.uint8, copy=False), k_small, iterations=1)
        else:
            grid_block = grid_raw.copy()

        # === 形態學清理（補洞→去孤點） ===
        if open_close_kernel and open_close_kernel >= 3 and (open_close_kernel % 2 == 1):
            Koc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_close_kernel, open_close_kernel))
            grid_block = cv2.morphologyEx(grid_block, cv2.MORPH_CLOSE, Koc)
            grid_block = cv2.morphologyEx(grid_block, cv2.MORPH_OPEN,  Koc)

        # ===（可選）把守護半圓寫進障礙圖（一般建議關閉，改在路徑驗證做動態檢查）===
        sx, sy = float(sensor_world_xy[0]), float(sensor_world_xy[1])
        sensor_rc = meters_to_cell_xy(sx, sy, ROI=ROI, GRID_SIZE=grid_size)  # None 表示 ROI 外

        # === 幾何自由圖（白=可行） ===
        free_geom = (1 - grid_block).astype(np.uint8)

        # === raytrace「看過才自由」 gating（真的用在規劃） ===
        if use_raytrace_gating and (sensor_rc is not None):
            max_cells = int(round(max_ray_range_m / max(x_res, y_res)))
            targets_rc = _make_targets_from_occ(grid_block, sensor_rc, max_range_cells=max_cells)
            free_ray = raytrace_free_from_sensor(
                grid_block=grid_block, sensor_rc=sensor_rc,
                targets_rc=targets_rc, max_range_m=max_ray_range_m
            )
        else:
            free_ray = np.ones_like(free_geom, np.uint8)

        free_plan = (free_geom & free_ray).astype(np.uint8)

        # === 出圖（白=可行；黑=障礙） ===
        img_gray  = (free_plan * 255).astype(np.uint8)
        img_color = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
        if draw_annot:
            step_pix = max(1, int(round(0.5 / max(x_res, y_res))))  # 每 0.5m 畫線
            for i in range(0, W, step_pix):
                cv2.line(img_color, (i, 0), (i, H-1), (180,180,180), 1)
            for i in range(0, H, step_pix):
                cv2.line(img_color, (0, i), (W-1, i), (180,180,180), 1)
            if sensor_rc is not None:
                cv2.circle(img_color, (sensor_rc[1], sensor_rc[0]), 3, (0,255,255), -1)

        _safe_log(step, f"hit={int(grid_raw.sum())}, obst={int(grid_block.sum())}, "
                        f"free_geom={(free_geom>0).sum()} cells, free_plan={(free_plan>0).sum()} cells")

        extras = dict(sensor_rc=(int(sensor_rc[0]), int(sensor_rc[1])) if sensor_rc else None,
                      free_geom=free_geom, free_ray=free_ray, free_plan=free_plan)
        return grid_raw, grid_block, img_color, extras

    except Exception as e:
        _safe_log_exc(step, e)
        raise


def raytrace_free_from_sensor_local(grid_block: np.ndarray,
                                    sensor_rc: tuple[int,int],
                                    targets_rc: np.ndarray,
                                    ROI: dict, GRID_SIZE: int,
                                    max_range_m: float) -> np.ndarray:
    """
    1=自由(看過且沿途無障礙)。使用 Bresenham 射線。
    射程用格解析度換算：step_m = max(x_res,y_res)
    """
    H, W = grid_block.shape
    free = np.zeros((H, W), np.uint8)
    if sensor_rc is None or targets_rc is None or len(targets_rc) == 0:
        return free
    sr, sc = int(sensor_rc[0]), int(sensor_rc[1])
    if not (0 <= sr < H and 0 <= sc < W):
        return free

    # 解析度 → 射程上限（格）
    x_res, y_res = _xy_res(ROI, GRID_SIZE)
    step_m = max(float(x_res), float(y_res))
    max_steps = max(1, int(max_range_m / step_m))
    r2_limit = max_steps * max_steps

    # 簡版 Bresenham
    def _line(r0, c0, r1, c1):
        dr = abs(r1 - r0); dc = abs(c1 - c0)
        sgn_r = 1 if r0 < r1 else -1
        sgn_c = 1 if c0 < c1 else -1
        err = dr - dc
        r, c = r0, c0
        out = []
        while True:
            out.append((r, c))
            if r == r1 and c == c1: break
            e2 = 2 * err
            if e2 > -dc: err -= dc; r += sgn_r
            if e2 <  dr: err += dr; c += sgn_c
        return out

    for (hr, hc) in targets_rc:
        dr = int(hr) - sr; dc = int(hc) - sc
        if (dr*dr + dc*dc) > r2_limit:
            continue
        steps = max(abs(dr), abs(dc))
        if steps <= 0:
            continue
        r1 = sr + dr; c1 = sc + dc
        line = _line(sr, sc, r1, c1)
        for (r, c) in line:
            if not (0 <= r < H and 0 <= c < W): break
            if grid_block[r, c] == 1:          # 撞到障礙就停
                break
            free[r, c] = 1
    return free


def pcd_to_occupancy_from_o3d_xy(
    pcd_xy: o3d.geometry.PointCloud,
    roi_xy: Dict,
    grid_size: int,
    **kwargs,
):
    """
    Convenience wrapper that reuses the X/Z implementation for XY projections.
    """
    roi_variant = {
        "x_min": roi_xy["x_min"],
        "x_max": roi_xy["x_max"],
        "y_min": roi_xy["y_min"],
        "y_max": roi_xy["y_max"],
    }
    return pcd_to_occupancy_from_o3d(
        pcd_xy,
        ROI=roi_variant,
        grid_size=grid_size,
        **kwargs,
    )


def xy_resolution(roi: Dict, grid_size: int) -> Tuple[float, float]:
    return _xy_res(roi, grid_size)
