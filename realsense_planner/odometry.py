from __future__ import annotations

from typing import Optional
import threading

from .robot_tcp import RobotTCPClient, RobotPose
from . import config as cfg


class OdometryClient:
    """
    向後兼容的里程計客戶端封裝。
    內部使用 RobotTCPClient 來處理連線和位姿追踪。
    
    注意：此類別已整合至 RobotTCPClient，建議直接使用 RobotTCPClient。
    此類別僅為向後兼容而保留。
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 8888):
        """
        初始化里程計客戶端
        
        Args:
            host: TCP 伺服器地址
            port: TCP 伺服器端口
        """
        # 內部使用 RobotTCPClient
        self._tcp_client = RobotTCPClient(
            ip=host,
            port=port,
            on_pos=None,
            on_ack=None,
            on_err=None,
            on_event=None
        )
        self._started = False

    def start(self):
        """啟動里程計客戶端"""
        if not self._started:
            self._tcp_client.connect()
            self._started = True

    def stop(self):
        """停止里程計客戶端"""
        if self._started:
            self._tcp_client.close()
            self._started = False

    def get_pose(self) -> RobotPose:
        """
        獲取當前機器人位姿
        
        Returns:
            RobotPose: 包含 x_m, y_m, heading_deg 的位姿資訊
        """
        return self._tcp_client.get_pose()

    def is_connected(self) -> bool:
        """
        返回是否曾經接收過里程計數據。
        即使斷線，只要曾接收過數據就返回 True，
        這樣會繼續使用最後已知的位姿而不是回到預設值。
        
        Returns:
            bool: 是否有里程計數據
        """
        return self._tcp_client.has_odometry_data()
    
    def get_connection_status(self) -> bool:
        """
        返回實際的 socket 連線狀態
        
        Returns:
            bool: 是否已連接
        """
        return self._tcp_client.connected()
    
    @property
    def host(self) -> str:
        """獲取主機地址"""
        return self._tcp_client.ip
    
    @property
    def port(self) -> int:
        """獲取端口"""
        return self._tcp_client.port
