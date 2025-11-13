from __future__ import annotations

import socket
import threading
from dataclasses import dataclass
from typing import Optional


@dataclass
class RobotPose:
    x_m: float = 0.0
    y_m: float = 0.0
    heading_deg: float = 90.0  # 世界座標下，預設正前方為 +Y


class OdometryClient:
    """
    連線至凱比 Android 端的 TCP 伺服器，解析 POS/POSM 訊息並提供最新位姿。
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 8888):
        self.host = host
        self.port = port
        self._pose = RobotPose()
        self._pose_lock = threading.Lock()
        self._conn_lock = threading.Lock()
        self._connected = False
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="OdometryClient", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        with self._conn_lock:
            self._connected = False

    def get_pose(self) -> RobotPose:
        with self._pose_lock:
            return RobotPose(self._pose.x_m, self._pose.y_m, self._pose.heading_deg)

    def _run(self):
        while not self._stop_event.is_set():
            try:
                with socket.create_connection((self.host, self.port), timeout=5.0) as sock:
                    with self._conn_lock:
                        self._connected = True
                    sock_file = sock.makefile("r")
                    while not self._stop_event.is_set():
                        line = sock_file.readline()
                        if not line:
                            break
                        self._handle_line(line.strip())
            except OSError:
                with self._conn_lock:
                    self._connected = False
                self._stop_event.wait(1.0)
            finally:
                if not self._stop_event.is_set():
                    with self._conn_lock:
                        self._connected = False

    def _handle_line(self, line: str):
        if line.startswith("POSM:"):
            payload = line[5:]
            parts = payload.split(",")
            if len(parts) >= 3:
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    heading = float(parts[2])
                    with self._pose_lock:
                        self._pose.x_m = x
                        self._pose.y_m = y
                        self._pose.heading_deg = heading
                except ValueError:
                    pass

    def is_connected(self) -> bool:
        with self._conn_lock:
            return self._connected
