from __future__ import annotations

import math
import threading
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from . import config as cfg
from .logging_utils import log

globals().update({name: getattr(cfg, name) for name in cfg.__all__})


def _h_octile(a: Tuple[int, int], b: Tuple[int, int], diag_cost: float = 1.41421356237) -> float:
    dr = abs(a[0] - b[0]); dc = abs(a[1] - b[1])
    D, D2 = 1.0, float(diag_cost)
    return (D * (dr + dc) + (D2 - 2 * D) * min(dr, dc))


def _h_manhattan(a: Tuple[int,int], b: Tuple[int,int]) -> float:
    return float(abs(a[0] - b[0]) + abs(a[1] - b[1]))


def _snap_to_nearest_free(grid_free: np.ndarray,
                          rc: Tuple[int, int],
                          allow_diagonal: bool = True) -> Optional[Tuple[int, int]]:
    """
    若 rc 在障礙上，回傳最近的可走格；找不到回 None。
    使用 BFS，對角是否允許由 allow_diagonal 決定。
    """
    H, W = grid_free.shape
    r0, c0 = int(rc[0]), int(rc[1])
    if 0 <= r0 < H and 0 <= c0 < W and grid_free[r0, c0] == 1:
        return (r0, c0)

    dirs = _DIRS8 if allow_diagonal else _DIRS4
    vis = np.zeros((H, W), np.uint8)
    q = deque()

    # 起始點若越界，從邊界投影到最近格
    r0 = int(np.clip(r0, 0, H - 1)); c0 = int(np.clip(c0, 0, W - 1))
    q.append((r0, c0)); vis[r0, c0] = 1

    while q:
        r, c = q.popleft()
        if grid_free[r, c] == 1:
            return (r, c)
        for dr, dc in dirs:
            r2, c2 = r + int(dr), c + int(dc)
            if 0 <= r2 < H and 0 <= c2 < W and not vis[r2, c2]:
                vis[r2, c2] = 1
                q.append((r2, c2))
    return None


def astar_opt(
    grid_free: np.ndarray,  # 1=可走, 0=障礙
    start: Tuple[int, int],
    goal: Tuple[int, int],
    *,
    allow_diagonal: bool = True,
    diag_cost: float = 1.41421356237,
    avoid_corner_cut: bool = True
) -> list:
    step = "astar_opt"
    try:
        H, W = grid_free.shape
        sr, sc = map(int, start); gr, gc = map(int, goal)

        if not (0 <= sr < H and 0 <= sc < W and 0 <= gr < H and 0 <= gc < W):
            _safe_log(step, "❌ start/goal 超出邊界")
            return []

        # 起點保證可走（避免安全圈把起點判黑）
        grid_free = grid_free.copy()
        grid_free[sr, sc] = 1

        if (sr, sc) == (gr, gc):
            return [(sr, sc)]
        if grid_free[gr, gc] == 0:
            # 終點在障礙上：由呼叫端決定是否吸附，這裡先回空
            return []

        # 移動/啟發
        if allow_diagonal:
            dirs = _DIRS8
            step_cost = np.ones(8, dtype=np.float32); step_cost[4:] = float(diag_cost)
            heur = (lambda p, q: _h_octile(p, q, diag_cost))
        else:
            dirs = _DIRS4
            step_cost = np.ones(4, dtype=np.float32)
            heur = _h_manhattan

        g = np.full((H, W), np.inf, dtype=np.float32)
        parent_dir = np.full((H, W), -1, dtype=np.int16)
        closed = np.zeros((H, W), dtype=bool)

        g[sr, sc] = 0.0
        h0 = heur((sr, sc), (gr, gc))
        open_heap = [(h0, h0, sr, sc)]
        heapq.heapify(open_heap)

        while open_heap:
            f, h, r, c = heapq.heappop(open_heap)
            if closed[r, c]:
                continue
            if (r, c) == (gr, gc):
                # 回溯路徑
                path = []
                rr, cc = r, c
                while not (rr == sr and cc == sc):
                    path.append((rr, cc))
                    k = parent_dir[rr, cc]
                    if k < 0:
                        return []
                    dr, dc = dirs[k]
                    rr -= int(dr); cc -= int(dc)
                path.append((sr, sc))
                path.reverse()
                return path

            closed[r, c] = True
            base_g = g[r, c]

            for k in range(len(dirs)):
                dr = int(dirs[k, 0]); dc = int(dirs[k, 1])
                r2, c2 = r + dr, c + dc
                if not (0 <= r2 < H and 0 <= c2 < W):
                    continue
                if grid_free[r2, c2] == 0:
                    continue

                # 嚴格避免切角：對角移動時，兩側任一為障礙即不通過
                if allow_diagonal and avoid_corner_cut and dr != 0 and dc != 0:
                    if grid_free[r, c + dc] == 0 or grid_free[r + dr, c] == 0:
                        continue

                g2 = base_g + step_cost[k]
                if g2 < g[r2, c2]:
                    g[r2, c2] = g2
                    parent_dir[r2, c2] = k
                    hh = heur((r2, c2), (gr, gc))
                    ff = float(g2 + hh)
                    heapq.heappush(open_heap, (ff, hh, r2, c2))

        _safe_log(step, "⚠️ 無可行路徑")
        return []
    except Exception:
        _safe_log_exc(step)
        return []


def draw_and_save_path(
    grid_block: np.ndarray,            # 1=不可行, 0=可走（幾何）
    vis_img_bgr: Optional[np.ndarray], # 作為底圖；若 None 或尺寸不符會自動產生白底
    start_rc: Tuple[int, int],
    goal_rc: Tuple[int, int],
    *,
    free_mask: Optional[np.ndarray] = None,      # 最好丟 extras["free_plan"]
    out_png: Optional[str] = None,
    out_csv: Optional[str] = None,
    ROI: Optional[dict] = None,                  # {x_min,x_max,y_min,y_max}
    grid_size: Optional[int] = None,
    cell_to_meters_fn: Optional[Callable[[int, int], Tuple[float, float]]] = None,  # 客製轉換
    allow_diagonal: bool = True,
    diag_cost: float = 1.41421356237,
    avoid_corner_cut: bool = True,
    snap_goal_to_nearest_free: bool = True
) -> np.ndarray:
    """
    - 若提供 free_mask（建議 extras["free_plan"]），規劃只在“看過的自由”內進行。
    - 起點會強制設成可走，避免被安全圈/去噪誤黑。
    - 終點若在障礙且允許吸附，會貼到最近可走格。
    """
    step = "draw_and_save_path"
    try:
        H, W = grid_block.shape
        base_free = (grid_block == 0).astype(np.uint8)
        if free_mask is not None and free_mask.shape == grid_block.shape:
            grid_free = (base_free & (free_mask > 0)).astype(np.uint8)
        else:
            if free_mask is not None and free_mask.shape != grid_block.shape:
                _safe_log(step, "⚠️ free_mask 尺寸不符，忽略 free_mask")
            grid_free = base_free

        sr, sc = int(start_rc[0]), int(start_rc[1])
        gr, gc = int(goal_rc[0]),  int(goal_rc[1])

        if not (0 <= sr < H and 0 <= sc < W and 0 <= gr < H and 0 <= gc < W):
            _safe_log(step, "❌ start/goal 超出邊界")
            img = vis_img_bgr if (vis_img_bgr is not None and vis_img_bgr.shape[:2] == (H, W)) \
                  else np.full((H, W, 3), 255, np.uint8)
            return img

        # 起點可走
        grid_free[sr, sc] = 1

        # 終點救援
        if grid_free[gr, gc] == 0 and snap_goal_to_nearest_free:
            ng = _snap_to_nearest_free(grid_free, (gr, gc), allow_diagonal=allow_diagonal)
            if ng is not None:
                gr, gc = ng

        path = astar_opt(grid_free, (sr, sc), (gr, gc),
                         allow_diagonal=allow_diagonal,
                         diag_cost=diag_cost,
                         avoid_corner_cut=avoid_corner_cut)

        # 底圖
        if vis_img_bgr is None or vis_img_bgr.shape[:2] != (H, W):
            img = np.full((H, W, 3), 255, np.uint8)  # 白底
        else:
            img = vis_img_bgr.copy()

        # 畫路徑/起終
        if len(path) > 0:
            for r, c in path:
                cv2.circle(img, (int(c), int(r)), 1, (0, 0, 255), -1)   # 路徑紅
            cv2.circle(img, (int(sc), int(sr)), 4, (0, 255, 255), -1)   # start 黃
            cv2.circle(img, (int(gc), int(gr)), 4, (0, 255,   0), -1)   # goal 綠
        else:
            _safe_log(step, "⚠️ 找不到路徑，僅輸出底圖")

        # 存圖
        if out_png:
            try:
                cv2.imwrite(out_png, img)
                _safe_log(step, f"存檔 {out_png}（路徑點數={len(path)}）")
            except Exception:
                _safe_log(step, f"⚠️ 無法存檔 {out_png}")

        # 存 CSV（世界 X,Y）
        if out_csv and len(path) > 0:
            def _cell_to_meters_xy(row: int, col: int) -> Optional[Tuple[float, float]]:
                if cell_to_meters_fn is not None:
                    return cell_to_meters_fn(row, col)
                if ROI is not None and grid_size is not None:
                    x_res = (ROI["x_max"] - ROI["x_min"]) / float(grid_size)
                    y_res = (ROI["y_max"] - ROI["y_min"]) / float(grid_size)
                    x = ROI["x_min"] + (col + 0.5) * x_res
                    y = ROI["y_max"] - (row + 0.5) * y_res
                    return (float(x), float(y))
                return None

            try:
                with open(out_csv, "w", encoding="utf-8") as f:
                    f.write("x_m,y_m\n")
                    for r, c in path:
                        pt = _cell_to_meters_xy(int(r), int(c))
                        if pt is not None:
                            f.write(f"{pt[0]:.6f},{pt[1]:.6f}\n")
                _safe_log(step, f"存檔 {out_csv}")
            except Exception:
                _safe_log(step, f"⚠️ 無法存 CSV {out_csv}")

        return img

    except Exception:
        _safe_log_exc(step)
        H, W = grid_block.shape[:2]
        return np.full((H, W, 3), 255, np.uint8)


def set_goal_rc(rc): _goal_rc[0] = rc


def get_goal_rc():   return _goal_rc[0]


def set_robot_pos_rc(rc): _robot_pos_rc[0] = rc


def get_robot_pos_rc():   return _robot_pos_rc[0]


def _on_mouse_set_goal(event, x, y, flags, userdata):
    if event == cv2.EVENT_LBUTTONDOWN:
        set_goal_rc((int(y), int(x)))
        log("mouse", f"🎯 設定新目標 grid_rc={(int(y), int(x))}")


def _normalize_deg(a: float) -> float:
    while a > 180: a -= 360
    while a < -180: a += 360
    return a


def world_yaw_deg_to_grid_heading(yaw_deg: float) -> float:
    # 世界座標：0=+X(東), 90=+Y(北) → 網格：0=北(+Y), 90=東(+X)
    return float(90.0 - yaw_deg)


def _heading_from_step(dr: int, dc: int) -> int:
    """
    0=北(+Y/up), 90=東(+X/right), 180=南, -90=西；對角 = ±45, ±135
    """
    if   dr == -1 and dc ==  0: return 0
    elif dr ==  1 and dc ==  0: return 180
    elif dr ==  0 and dc ==  1: return 90
    elif dr ==  0 and dc == -1: return -90
    else:
        ang = math.degrees(math.atan2(dc, -dr))  # row 向上是負
        dirs = [0, 45, 90, 135, 180, -135, -90, -45]
        ang = min(dirs, key=lambda k: abs(_normalize_deg(ang - k)))
        return int(ang)


def _segment_path(path_rc, allow_diagonal: bool = True):
    """
    回傳 list[(heading_deg, cells_count, is_diag)]
    """
    if not path_rc or len(path_rc) < 2:
        return []
    segs = []
    i = 1
    while i < len(path_rc):
        r0, c0 = path_rc[i-1]
        r1, c1 = path_rc[i]
        dr, dc = r1 - r0, c1 - c0
        hd = _heading_from_step(dr, dc)
        cnt = 1
        j = i + 1
        while j < len(path_rc):
            pr, pc = path_rc[j-1]
            nr, nc = path_rc[j]
            dr2, dc2 = nr - pr, nc - pc
            if _heading_from_step(dr2, dc2) != hd:
                break
            cnt += 1
            j += 1
        is_diag = (abs(hd) in (45, 135))
        if not allow_diagonal and is_diag:
            # 理論上 A* 若禁對角不會到此；保險切段
            for _ in range(cnt):
                segs.append((0, 1, False))
        else:
            segs.append((hd, cnt, is_diag))
        i = j
    return segs


def path_to_commands(path_rc, *, cell_m: float, start_heading_deg: float = 0.0, allow_diagonal: bool = True):
    """
    start_heading_deg：請給「網格角度」（0=+Y、90=+X）。若手邊是世界角度，先用 world_yaw_deg_to_grid_heading() 轉。
    回傳：["turn:+/-deg", "move:meters", ...]
    """
    cmds = []
    heading = int(round(start_heading_deg))
    for hd, cnt, is_diag in _segment_path(path_rc, allow_diagonal=allow_diagonal):
        rot = _normalize_deg(hd - heading)
        if abs(rot) > 1:
            cmds.append(f"turn:{int(round(rot))}")
            heading = hd
        step_len = (math.sqrt(2.0) * cell_m) if is_diag else cell_m
        dist = max(0.0, cnt * step_len)
        if dist > 1e-4:
            cmds.append(f"move:{dist:.3f}")
    return cmds


def wait_reach_cell(tgt_rc: Tuple[int,int], timeout_s: float = 6.0, tol_cells: int = 0) -> bool:
    t0 = time.time()
    last_rc = None
    while time.time() - t0 <= timeout_s:
        rc = get_robot_pos_rc()
        if rc is not None:
            last_rc = rc
            if abs(rc[0]-tgt_rc[0]) <= tol_cells and abs(rc[1]-tgt_rc[1]) <= tol_cells:
                return True
        time.sleep(0.05)
    print(f"[nav] wait reach timeout. last_rc={last_rc}, tgt={tgt_rc}")
    return False


def _set_goal_world(x_m: float, y_m: float, ROI: dict, GRID_SIZE: int):
    rc = _meters_to_cell_xy(x_m, y_m, ROI, GRID_SIZE)
    if rc is None: return None
    H = W = int(GRID_SIZE)
    r, c = rc
    if 0 <= r < H and 0 <= c < W:
        set_goal_rc((r, c))
        return (r, c)
    return None
