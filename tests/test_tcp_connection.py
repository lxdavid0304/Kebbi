#!/usr/bin/env python3
"""
測試 TCP 連線到機器人的腳本
"""
from realsense_planner.robot_tcp import RobotTCPClient
from realsense_planner import config as cfg
import time

def test_tcp_connection():
    print(f"嘗試連線到機器人: {cfg.ROBOT_TCP_IP}:{cfg.ROBOT_TCP_PORT}")
    print("-" * 50)
    
    tcp = RobotTCPClient(cfg.ROBOT_TCP_IP, cfg.ROBOT_TCP_PORT)
    tcp.connect()
    
    # 等待連線建立
    print("等待連線建立...")
    for i in range(10):
        time.sleep(1)
        if tcp.connected():
            print(f"✅ TCP 連線成功！")
            break
        print(f"  等待中... ({i+1}/10)")
    else:
        print("❌ TCP 連線失敗")
        print("\n請檢查：")
        print(f"1. 機器人 IP 是否正確: {cfg.ROBOT_TCP_IP}")
        print(f"2. 機器人 Port 是否正確: {cfg.ROBOT_TCP_PORT}")
        print("3. 機器人是否已啟動 TCP 服務")
        print("4. 網路連線是否正常")
        tcp.close()
        return False
    
    # 測試發送指令
    print("\n測試發送指令...")
    try:
        tcp.send_line("stop")
        print("✅ 發送 'stop' 指令成功")
    except Exception as e:
        print(f"❌ 發送指令失敗: {e}")
        tcp.close()
        return False
    
    tcp.close()
    print("\n✅ 測試完成！TCP 連線正常")
    return True

if __name__ == "__main__":
    test_tcp_connection()