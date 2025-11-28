#!/usr/bin/env python3
"""
診斷工具：檢查配置和連線狀態
"""
import sys
import socket

print("=" * 60)
print("機器人配置診斷工具")
print("=" * 60)

# 1. 檢查模組導入路徑
print("\n[1] 檢查模組導入...")
try:
    from realsense_planner import config as cfg
    print(f"✅ 成功從 realsense_planner 導入 config")
    print(f"   模組位置: {cfg.__file__ if hasattr(cfg, '__file__') else '內建模組'}")
except ImportError as e:
    print(f"❌ 無法從 realsense_planner 導入: {e}")
    try:
        from Kebbi.realsense_planner import config as cfg
        print(f"✅ 成功從 Kebbi.realsense_planner 導入 config")
        print(f"   模組位置: {cfg.__file__ if hasattr(cfg, '__file__') else '內建模組'}")
    except ImportError as e2:
        print(f"❌ 無法從 Kebbi.realsense_planner 導入: {e2}")
        sys.exit(1)

# 2. 顯示當前配置
print("\n[2] 當前配置值:")
print(f"   ROBOT_TCP_IP   = {cfg.ROBOT_TCP_IP}")
print(f"   ROBOT_TCP_PORT = {cfg.ROBOT_TCP_PORT}")
print(f"   ROBOT_START    = {cfg.ROBOT_START}")
print(f"   ROBOT_INIT_YAW = {cfg.ROBOT_INIT_YAW_DEG}°")

# 3. 測試網路連線
print("\n[3] 測試 TCP 連線...")
print(f"   目標: {cfg.ROBOT_TCP_IP}:{cfg.ROBOT_TCP_PORT}")

def test_socket_connection(ip, port, timeout=5):
    """測試 socket 連線"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((ip, port))
        sock.close()
        return result == 0
    except Exception as e:
        print(f"   錯誤: {e}")
        return False

if test_socket_connection(cfg.ROBOT_TCP_IP, cfg.ROBOT_TCP_PORT):
    print(f"   ✅ TCP 連接埠可達！")
else:
    print(f"   ❌ TCP 連接埠無法連線")
    print(f"\n   可能原因:")
    print(f"   1. IP 地址錯誤（當前: {cfg.ROBOT_TCP_IP}）")
    print(f"   2. 端口錯誤（當前: {cfg.ROBOT_TCP_PORT}）")
    print(f"   3. 機器人未啟動或 TCP 服務未運行")
    print(f"   4. 防火牆阻擋")
    print(f"   5. 網路問題")

# 4. 測試完整的 RobotTCPClient
print("\n[4] 測試 RobotTCPClient...")
try:
    from realsense_planner.robot_tcp import RobotTCPClient
except ImportError:
    try:
        from Kebbi.realsense_planner.robot_tcp import RobotTCPClient
    except ImportError as e:
        print(f"   ❌ 無法導入 RobotTCPClient: {e}")
        sys.exit(1)

import time

tcp = RobotTCPClient(cfg.ROBOT_TCP_IP, cfg.ROBOT_TCP_PORT)
print(f"   啟動連線...")
tcp.connect()

# 等待連線
for i in range(10):
    time.sleep(0.5)
    if tcp.connected():
        print(f"   ✅ RobotTCPClient 連線成功！（{i*0.5:.1f}秒）")
        break
    if i % 2 == 0:
        print(f"   ⏳ 等待連線... ({i*0.5:.1f}秒)")
else:
    print(f"   ❌ RobotTCPClient 連線失敗（超時 5 秒）")

tcp.close()

# 5. 修改配置說明
print("\n[5] 如何修改配置:")
print(f"   編輯文件: realsense_planner/config.py 或 Kebbi/realsense_planner/config.py")
print(f"   修改以下行:")
print(f'   ROBOT_TCP_IP = "您的機器人IP"  # 例如: "192.168.1.100"')
print(f'   ROBOT_TCP_PORT = 您的端口號     # 例如: 8888')

print("\n" + "=" * 60)
print("診斷完成")
print("=" * 60)