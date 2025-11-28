#!/usr/bin/env python3
"""
測試 TCP 命令發送工具
用於確認機器人是否能正確接收並執行命令
"""

from realsense_planner import config as cfg
from realsense_planner.robot_tcp import RobotTCPClient
import time

def test_commands():
    print(f"連線到 {cfg.ROBOT_TCP_IP}:{cfg.ROBOT_TCP_PORT}")
    
    tcp = RobotTCPClient(cfg.ROBOT_TCP_IP, cfg.ROBOT_TCP_PORT)
    
    def on_ack(msg):
        print(f"[ACK] {msg}")
    
    def on_err(msg):
        print(f"[ERR] {msg}")
    
    def on_event(msg):
        print(f"[EVENT] {msg}")
    
    tcp._on_ack = on_ack
    tcp._on_err = on_err
    tcp._on_event = on_event
    
    tcp.connect()
    
    print("等待連線...")
    time.sleep(2)
    
    if not tcp.connected():
        print("❌ 連線失敗")
        return
    
    print("✅ 連線成功")
    
    commands = [
        ("forward", "前進"),
        ("stop", "停止"),
        ("backward", "後退"),
        ("stop", "停止"),
        ("left", "左轉"),
        ("stop", "停止"),
        ("right", "右轉"),
        ("stop", "停止"),
    ]
    
    print("\n開始測試命令發送...")
    print("請觀察機器人是否有反應\n")
    
    for cmd, desc in commands:
        try:
            print(f"發送: {cmd} ({desc})")
            tcp.send_line(cmd)
            time.sleep(2)  # 等待 2 秒觀察反應
        except Exception as e:
            print(f"❌ 發送失敗: {e}")
            break
    
    print("\n測試完成")
    tcp.close()

if __name__ == "__main__":
    test_commands()