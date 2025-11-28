#!/usr/bin/env python3
# client.py – TCP 控制 Kebbi 的小幫手（含 sleep / wake / always:on|off）

import socket
import time
import argparse
import os
import sys

DEFAULT_IP   = os.getenv("KEBBI_IP",   "172.20.10.8")
DEFAULT_PORT = int(os.getenv("KEBBI_PORT", "8888"))

# --- 指令速查表 -----------------------------------------------------------
COMMAND_SAMPLES = [
    ("sleep",                "進入軟睡眠（停聽/隱臉/鎖輪）"),
    ("wake",                 "退出軟睡眠（顯臉/解鎖），可選再 startRecognize"),
    ("always:on",            "開啟 Always Wakeup（熱詞常駐可喚醒）"),
    ("always:off",           "關閉 Always Wakeup（不建議常用）"),
    ("forward",              "連續前進（raw speed），用 stop 結束"),
    ("backward   (back)",    "連續後退"),
    ("left",                 "原地左轉"),
    ("right",                "原地右轉"),
    ("stop",                 "馬上停止 (move=0 & turn=0)"),
    ("forwardTime:3.0",      "前進 3 秒 → 自動 stop"),
    ("backTime:1.5",         "後退 1.5 秒 → 自動 stop"),
    ("leftTime:2.0",         "左轉 2 秒 → 自動 stop"),
    ("rightTime:0.8",        "右轉 0.8 秒 → 自動 stop"),
    ("move:0.5",             "直行 0.5 公尺（以常速換秒數）"),
    ("turn:-45",             "原地左轉 45°"),
    ("goto:1.0,0.5",         "導航到相對座標 (x,y)"),
    ("forward;left;forward", "分號可串多指令（會逐一送出）"),
]

# --- 小捷徑（輸入→展開成多條實際指令） -----------------------------------
# 例：awake  -> ["always:on", "wake"]
SHORTCUTS = {
    "awake":        ["always:on", "wake"],   # 最常用：保持可喚醒並退出軟睡
    "stay":         ["always:on"],           # 只開 Always Wakeup
    "nap":          ["sleep"],               # 小睡
    "dnd":          ["always:off"],          # Do-Not-Disturb（通常不建議）
}

def expand_shortcuts(raw: str):
    key = raw.strip().lower()
    if key in SHORTCUTS:
        return SHORTCUTS[key][:]
    # 兼容大小寫 + 去頭尾空白
    return [raw.strip()]

# -------------------------------------------------------------------------

def print_banner(ip: str, port: int) -> None:
    print("\n========== Kebbi TCP Client ==========")
    print(f"連線目標：{ip}:{port}")
    print("可用指令範例：")
    for ex, desc in COMMAND_SAMPLES:
        print(f"  {ex:<20} # {desc}")
    print("\n另外支援小捷徑：awake / stay / nap / dnd")
    print("輸入 help 重新顯示這些範例，輸入 exit 離開\n")

def send_cmd(ip: str, port: int, cmd: str) -> None:
    """開 TCP 送單條指令"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(3.0)
            s.connect((ip, port))
            s.sendall((cmd + "\n").encode("utf-8"))
            print(f"✅ 已送出 → {cmd}")
    except Exception as e:
        print(f"❌ 連線或傳送失敗：{e}")

# -------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Kebbi TCP Client")
    parser.add_argument("--ip",   default=DEFAULT_IP,   help="Kebbi IP（預設讀 KEBBI_IP 或 172.20.10.8）")
    parser.add_argument("--port", default=DEFAULT_PORT, type=int, help="TCP 連接埠（預設 8888）")
    parser.add_argument("-d", "--delay", default=0.2, type=float, help="批次指令之間的間隔秒數")
    args = parser.parse_args()

    print_banner(args.ip, args.port)

    while True:
        try:
            line = input(">> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nBye~")
            break

        if not line:
            continue
        low = line.lower()
        if low in ("exit", "quit"):
            break
        if low == "help":
            print_banner(args.ip, args.port)
            continue

        # 若只輸入座標「x,y」自動補 goto:
        if "," in line and not low.startswith("goto:"):
            line = "goto:" + line

        # 以分號拆多指令 → 先展開捷徑 → 逐一送
        pieces = [c.strip() for c in line.split(";") if c.strip()]
        queue = []
        for p in pieces:
            queue.extend(expand_shortcuts(p))

        for cmd in queue:
            send_cmd(args.ip, args.port, cmd)
            time.sleep(args.delay)

if __name__ == "__main__":
    main()


