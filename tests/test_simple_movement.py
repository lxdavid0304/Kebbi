#!/usr/bin/env python3
"""
簡單的移動測試腳本 - 用於診斷TCP指令問題
"""
import socket
import time
import threading

ROBOT_IP = "172.20.10.8"
ROBOT_PORT = 8888

def receive_messages(sock):
    """持續接收並顯示來自機器人的訊息"""
    buffer = b""
    while True:
        try:
            data = sock.recv(1024)
            if not data:
                print("❌ 連線已斷開")
                break
            
            buffer += data
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                msg = line.decode("utf-8", errors="ignore").strip()
                if msg:
                    print(f"📩 收到回應: {msg}")
        except Exception as e:
            print(f"❌ 接收錯誤: {e}")
            break

def send_command(sock, cmd):
    """發送單個指令"""
    try:
        sock.sendall((cmd + "\n").encode("utf-8"))
        print(f"📤 已發送: {cmd}")
        return True
    except Exception as e:
        print(f"❌ 發送失敗: {e}")
        return False

def main():
    print("=" * 60)
    print("🤖 Kebbi 機器人移動測試")
    print("=" * 60)
    print(f"連線目標: {ROBOT_IP}:{ROBOT_PORT}")
    print()
    
    try:
        # 建立TCP連線
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        print("🔌 正在連線...")
        sock.connect((ROBOT_IP, ROBOT_PORT))
        print("✅ TCP連線成功！")
        print()
        
        # 啟動接收執行緒
        sock.settimeout(None)
        rx_thread = threading.Thread(target=receive_messages, args=(sock,), daemon=True)
        rx_thread.start()
        
        # 等待初始訊息
        time.sleep(1)
        
        # 測試指令序列
        print("🧪 開始測試基本移動指令...")
        print("-" * 60)
        
        tests = [
            ("stop", "確保初始狀態停止", 1),
            ("forward", "開始前進", 2),
            ("stop", "停止前進", 1),
            ("backward", "開始後退", 2),
            ("stop", "停止後退", 1),
            ("left", "開始左轉", 2),
            ("stop", "停止左轉", 1),
            ("right", "開始右轉", 2),
            ("stop", "停止右轉", 1),
        ]
        
        for cmd, desc, wait_sec in tests:
            print(f"\n📋 測試: {desc}")
            if send_command(sock, cmd):
                time.sleep(wait_sec)
            else:
                break
        
        print("\n" + "=" * 60)
        print("✅ 測試完成！")
        print()
        print("💡 診斷建議:")
        print("1. 如果看到 'HELLO:RobotReady' - Android端已正常啟動")
        print("2. 如果看到 'ACK:xxx' - Android端有接收並處理指令")
        print("3. 如果機器人沒有移動但有ACK:")
        print("   - 檢查Android端的MainActivity是否正確調用RobotMotionController")
        print("   - 檢查NuwaRobotAPI是否成功初始化")
        print("   - 查看Android Logcat日誌")
        print("4. 如果完全沒有回應:")
        print("   - 確認Android app正在前台運行")
        print("   - 確認TCP Server已啟動 (port 8888)")
        
    except socket.timeout:
        print("❌ 連線超時！")
        print("   請確認:")
        print("   1. 機器人IP是否正確")
        print("   2. 機器人是否在同一網路")
        print("   3. Android app是否正在運行")
    except ConnectionRefusedError:
        print("❌ 連線被拒絕！")
        print("   請確認:")
        print("   1. Android app是否正在運行")
        print("   2. TCP Server是否已啟動")
    except Exception as e:
        print(f"❌ 錯誤: {e}")
    finally:
        try:
            sock.close()
        except:
            pass

if __name__ == "__main__":
    main()