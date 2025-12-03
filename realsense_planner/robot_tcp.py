from __future__ import annotations

import socket
import threading
import time
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Optional

from . import config as cfg
from .logging_utils import log
from .planner import _segment_path, path_to_commands, wait_reach_cell

globals().update({name: getattr(cfg, name) for name in cfg.__all__})


@dataclass
class RobotPose:
    """機器人位姿資訊（x/y：公尺，heading：度）"""
    x_m: float = 0.0
    y_m: float = 0.0
    heading_deg: float = 0.0  # 里程計座標下預設正前方為 +X


def _normalize_heading_deg(angle: float) -> float:
    """Wrap heading to [0, 360)。"""
    return angle % 360.0


class RobotTCPClient:
    def __init__(self, ip: str, port: int, on_pos=None, on_ack=None, on_err=None, on_event=None):
        self.ip, self.port = ip, port
        self.sock = None
        self.sock_file = None  # 用於 flush 的文件對象
        self.rx_thread = None
        self.tx_lock = threading.Lock()
        self.running = False
        self._buffer = b""
        self._on_pos = on_pos
        self._on_ack = on_ack
        self._on_err = on_err
        self._on_event = on_event
        self._connected = threading.Event()
        self._hb_thread = None
        self._last_pong_ts = 0.0
        
        # 里程計位姿追踪（合併自 OdometryClient）
        self._pose = RobotPose()
        self._pose_lock = threading.Lock()
        self._has_received_data = False  # 追蹤是否曾接收過里程計數據
        self.start_pos_x = 0.0
        self.start_pos_y = 0.0
        self.start_heading = 0.0
        self.has_init_pos = False

    # -------- 連線與接收迴圈 --------
    def connect(self, retry_sec=2.0):
        self.running = True

        def _open_socket():
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except Exception:
                pass
            try:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            except Exception:
                pass
            s.settimeout(5.0)
            s.connect((self.ip, self.port))
            s.settimeout(None)
            return s

        def _rx_loop():
            while self.running:
                try:
                    if self.sock is None:
                        try:
                           
                            # 創建文件對象用於 flush
                            self.sock = _open_socket()
                            self.sock_file = self.sock.makefile('wb', buffering=0)
                            
                            self._connected.set()
                            self._last_pong_ts = time.time()
                            print(f"[tcp] ✅ connected to {self.ip}:{self.port}")
                        except Exception as e:
                            print(f"[tcp] connect failed: {e}")
                            self._connected.clear()
                            time.sleep(retry_sec)
                            continue

                    data = self.sock.recv(4096)
                    if not data:
                        raise ConnectionError("peer closed")

                    self._buffer += data
                    # 防止緩衝暴增
                    if len(self._buffer) > 1_000_000:
                        self._buffer = self._buffer[-8192:]

                    while b"\n" in self._buffer:
                        line, self._buffer = self._buffer.split(b"\n", 1)
                        msg = line.decode("utf-8", errors="ignore").strip()

                        if msg == "PONG":
                            self._last_pong_ts = time.time()
                            continue
                        if msg.startswith("POS:"):
                            payload = msg[4:].strip()
                            # 兩種格式：POS:r,c 或 POS:x,y[,yaw]
                            try:
                                toks = [t.strip() for t in payload.split(",")]
                                if len(toks) >= 3:  # x,y,yaw
                                    x = float(toks[0]); y = float(toks[1])
                                    yaw = float(toks[2])
                                    if self._on_pos: self._on_pos(("xy", (x, y, yaw)))
                                elif len(toks) == 2:
                                    a = float(toks[0]); b = float(toks[1])
                                    if toks[0].isdigit() and toks[1].isdigit():
                                        if self._on_pos: self._on_pos(("rc", (int(a), int(b))))
                                    else:
                                        if self._on_pos: self._on_pos(("xy", (a, b, None)))
                            except Exception:
                                pass
                            continue
                        if msg.startswith("POSM:"):
                            # 處理里程計訊息（合併自 OdometryClient）
                            payload = msg[5:].strip()
                            parts = payload.split(",")
                            if len(parts) >= 3:
                                try:
                                    x = float(parts[0])
                                    y = float(parts[1])
                                    heading = float(parts[2])
                                    if not self.has_init_pos:
                                        self.start_pos_x = x
                                        self.start_pos_y = y
                                        self.start_heading = heading
                                        self.has_init_pos = True
                                        print(f"[tcp-odom] 初始位置設定: x={x}, y={y}, heading={heading}")
                                    with self._pose_lock:
                                        self._pose.x_m = x - self.start_pos_x
                                        self._pose.y_m = y - self.start_pos_y
                                        self._pose.heading_deg = _normalize_heading_deg(heading - self.start_heading)
                                        self._has_received_data = True
                                    print(f"[tcp-odom] 位姿更新: x={self._pose.x_m:.3f}, y={self._pose.y_m:.3f}, heading={self._pose.heading_deg:.1f}°")
                                except ValueError as e:
                                    print(f"[tcp-odom] POSM 解析錯誤: {e}")
                            continue
                        if msg.startswith("ACK:"):
                            if self._on_ack: self._on_ack(msg[4:].strip()); continue
                        if msg.startswith("ERR:"):
                            if self._on_err: self._on_err(msg[4:].strip()); continue
                        if msg == "EXPLORE_DONE":
                            if self._on_event: self._on_event(msg)
                            continue
                        # 其他訊息交給外部回呼（例如 HUMAN_DIST）
                        if self._on_event:
                            try:
                                self._on_event(msg)
                                continue
                            except Exception:
                                pass
                        print("[tcp] <", msg)

                    # 心跳超時（例如 10 秒沒有 PONG）
                    if time.time() - self._last_pong_ts > 12.0:
                        raise TimeoutError("heartbeat lost")
                except Exception as e:
                    print(f"[tcp] rx error: {e}")
                    self._connected.clear()
                    try:
                        if self.sock_file:
                            self.sock_file.close()
                    except Exception:
                        pass
                    try:
                        if self.sock:
                            self.sock.close()
                    except Exception:
                        pass
                    self.sock = None
                    self.sock_file = None
                    time.sleep(retry_sec)

        self.rx_thread = threading.Thread(target=_rx_loop, daemon=True)
        self.rx_thread.start()
        self._start_heartbeat()

    # -------- 心跳發送 --------
    def _start_heartbeat(self, interval=2.0):
        if self._hb_thread and self._hb_thread.is_alive():
            return

        def _hb():
            while self.running:
                try:
                    if self.connected():
                        with self.tx_lock:
                            self.sock.sendall(b"PING\n")
                    time.sleep(interval)
                except Exception:
                    time.sleep(interval)

        self._hb_thread = threading.Thread(target=_hb, daemon=True)
        self._hb_thread.start()

    def connected(self) -> bool:
        return self._connected.is_set()

    # -------- 發送一行 --------
    def send_line(self, text: str):
        data = (text.strip() + "\n").encode("utf-8")
        with self.tx_lock:
            if not self.connected():
                raise ConnectionError("tcp not connected")
            try:
                if self.sock_file:
                    # 使用文件對象寫入並刷新
                    self.sock_file.write(data)
                    self.sock_file.flush()
                else:
                    # 備用方案：直接使用 socket
                    self.sock.sendall(data)
                print("[cmd] >",data)
            except Exception:
                self._connected.clear()
                try:
                    if self.sock_file:
                        self.sock_file.close()
                except Exception:
                    pass
                try:
                    if self.sock:
                        self.sock.close()
                except Exception:
                    pass
                self.sock = None
                self.sock_file = None
                raise

    # -------- 關閉 --------
    def close(self):
        self.running = False
        try:
            if self.sock_file:
                self.sock_file.close()
        except Exception:
            pass
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass
        self.sock = None
        self.sock_file = None
        self._connected.clear()

    # -------- 里程計功能（合併自 OdometryClient）--------
    def get_pose(self) -> RobotPose:
        """獲取當前機器人位姿"""
        with self._pose_lock:
            return RobotPose(self._pose.x_m, self._pose.y_m, self._pose.heading_deg)
    
    def has_odometry_data(self) -> bool:
        """
        返回是否曾經接收過里程計數據。
        即使斷線，只要曾接收過數據就返回 True，
        這樣會繼續使用最後已知的位姿而不是回到預設值。
        """
        with self._pose_lock:
            return self._has_received_data
    
    def reset_odometry(self):
        """重置里程計，下次接收 POSM 時會重新設定起始點"""
        with self._pose_lock:
            self.has_init_pos = False
            self.start_pos_x = 0.0
            self.start_pos_y = 0.0
            self.start_heading = 0.0
            print("[tcp-odom] 里程計已重置")


def execute_path_over_tcp(path_rc, tcp, *, start_heading_deg: float = 0.0, allow_diagonal: bool = True) -> bool:
    """
    start_heading_deg：給「網格方位」用（若要從世界 yaw 轉，可用 world_yaw_deg_to_grid_heading）。
    """
    if len(path_rc) < 2:
        return True

    # 1) 產生指令序列（含轉向與移動距離）
    cmds = path_to_commands(
        path_rc,
        cell_m=float(cfg.CELL_M),
        start_heading_deg=start_heading_deg,
        allow_diagonal=allow_diagonal,
    )
    print(f"[nav] cmds={cmds}")

    # 2) 依段落執行，每段移動完等待抵達
    seg_end_indices = []
    acc = 0
    for _, cnt, _ in _segment_path(path_rc, allow_diagonal=allow_diagonal):
        acc += cnt
        seg_end_indices.append(acc)

    cmd_idx = 0
    for seg_end in seg_end_indices:
        # a) TURN（若存在）
        if cmd_idx < len(cmds) and cmds[cmd_idx].startswith("turn:"):
            tcp.send_line(cmds[cmd_idx])
            cmd_idx += 1
            time.sleep(0.2)  # Android 端可能回 ACK

        # b) MOVE
        if cmd_idx < len(cmds) and cmds[cmd_idx].startswith("move:"):
            tcp.send_line(cmds[cmd_idx])
            cmd_idx += 1
            tgt_rc = path_rc[min(seg_end, len(path_rc) - 1)]
            ok = wait_reach_cell(tgt_rc, timeout_s=8.0, tol_cells=0)
            if not ok:
                try:
                    tcp.send_line("stop")
                except Exception:
                    pass
                return False
    return True
