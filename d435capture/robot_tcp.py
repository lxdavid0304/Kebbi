from __future__ import annotations

import socket
import threading
import time
from typing import Iterable, List, Sequence

from . import config as cfg
from .logging_utils import log
from .planner import _segment_path, path_to_commands, wait_reach_cell

globals().update({name: getattr(cfg, name) for name in cfg.__all__})


class RobotTCPClient:
    def __init__(self, ip: str, port: int, on_pos=None, on_ack=None, on_err=None, on_event=None):
        self.ip, self.port = ip, port
        self.sock = None
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
                            self.sock = _open_socket()
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
                    # 防止無限累積
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
                            # 支援兩種：POS:r,c 或 POS:x,y[,yaw]
                            try:
                                toks = [t.strip() for t in payload.split(",")]
                                if len(toks) >= 3:  # x,y,yaw
                                    x = float(toks[0]); y = float(toks[1])
                                    yaw = float(toks[2])
                                    if self._on_pos: self._on_pos(("xy", (x, y, yaw)))
                                elif len(toks) == 2:
                                    a = float(toks[0]); b = float(toks[1])
                                    # 推測整數=格座標；浮點=世界座標
                                    if toks[0].isdigit() and toks[1].isdigit():
                                        if self._on_pos: self._on_pos(("rc", (int(a), int(b))))
                                    else:
                                        if self._on_pos: self._on_pos(("xy", (a, b, None)))
                            except Exception:
                                pass
                            continue
                        if msg.startswith("ACK:"):
                            if self._on_ack: self._on_ack(msg[4:].strip()); continue
                        if msg.startswith("ERR:"):
                            if self._on_err: self._on_err(msg[4:].strip()); continue
                        if msg == "EXPLORE_DONE":
                            if self._on_event: self._on_event(msg)
                            continue
                        # 其他文字就打印
                        print("[tcp] <", msg)

                    # 心跳超時（例如 10 秒沒收到 PONG）
                    if time.time() - self._last_pong_ts > 12.0:
                        raise TimeoutError("heartbeat lost")
                except Exception as e:
                    print(f"[tcp] rx error: {e}")
                    self._connected.clear()
                    try:
                        if self.sock: self.sock.close()
                    except: pass
                    self.sock = None
                    time.sleep(retry_sec)

        self.rx_thread = threading.Thread(target=_rx_loop, daemon=True)
        self.rx_thread.start()

        # 啟動心跳送出執行緒
        self._start_heartbeat()

    # -------- 心跳送出 --------
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
                except Exception as e:
                    # 讓 rx 線程負責重連
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
                self.sock.sendall(data)
            except Exception as e:
                self._connected.clear()
                try:
                    if self.sock: self.sock.close()
                except: pass
                self.sock = None
                raise

    # -------- 關閉 --------
    def close(self):
        self.running = False
        try:
            if self.sock: self.sock.close()
        except: pass
        self.sock = None
        self._connected.clear()


def execute_path_over_tcp(path_rc, tcp, *, start_heading_deg: float = 0.0, allow_diagonal: bool = True) -> bool:
    """
    start_heading_deg：請給「網格角度」；若妳的全域使用世界角度(0=+X,90=+Y)，請先轉成網格角度。
    例：start_heading_deg = world_yaw_deg_to_grid_heading(ROBOT_INIT_YAW_DEG)
    """
    if len(path_rc) < 2:
        return True

    # 1) 產生指令序列（含對角距離 √2）
    cmds = path_to_commands(
        path_rc,
        cell_m=float(cfg.CELL_M),
        start_heading_deg=start_heading_deg,
        allow_diagonal=allow_diagonal,
    )
    print(f"[nav] cmds={cmds}")

    # 2) 依段落執行；每段末等待抵達該段終點格
    seg_end_indices = []
    acc = 0
    for _, cnt, _ in _segment_path(path_rc, allow_diagonal=allow_diagonal):
        acc += cnt
        seg_end_indices.append(acc)

    cmd_idx = 0
    for seg_end in seg_end_indices:
        # a) TURN（如有）
        if cmd_idx < len(cmds) and cmds[cmd_idx].startswith("turn:"):
            tcp.send_line(cmds[cmd_idx])
            cmd_idx += 1
            time.sleep(0.2)  # 若 Android 有 ACK:TURN_DONE 可改成等待 ACK

        # b) MOVE
        if cmd_idx < len(cmds) and cmds[cmd_idx].startswith("move:"):
            tcp.send_line(cmds[cmd_idx])
            cmd_idx += 1
            tgt_rc = path_rc[min(seg_end, len(path_rc)-1)]
            ok = wait_reach_cell(tgt_rc, timeout_s=8.0, tol_cells=0)
            if not ok:
                try:
                    tcp.send_line("stop")
                except Exception:
                    pass
                return False
    return True
