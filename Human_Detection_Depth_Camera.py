#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MaixSense-A010 CLI wrapper that wires controller, detection, packet parsing, and visualization modules.
"""

from __future__ import annotations

import argparse
import sys
import time

import cv2

from maixsense import MaixSenseController, PersonDetectionParams


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MaixSense-A010 串口監控工具")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="串口設備路徑")
    parser.add_argument("--baudrate", type=int, default=57600, help="串口波特率")
    parser.add_argument("--fps", type=int, default=4, help="相機輸出 FPS")
    parser.add_argument("--min-distance", type=float, default=500.0, help="偵測最小距離 (mm)")
    parser.add_argument("--max-distance", type=float, default=1500.0, help="偵測最大距離 (mm)")
    parser.add_argument("--min-area", type=int, default=500, help="人形最小像素面積")
    parser.add_argument("--verbose", action="store_true", help="輸出詳細偵錯訊息")
    return parser


def main(argv: list[str] | None = None):
    args = build_arg_parser().parse_args(argv)
    detection_params = PersonDetectionParams(
        min_distance_mm=args.min_distance,
        max_distance_mm=args.max_distance,
        min_area=args.min_area,
    )
    controller = MaixSenseController(
        port=args.port,
        baudrate=args.baudrate,
        verbose=args.verbose,
        detection_params=detection_params,
    )

    monitor_thread = None
    print("MaixSense-A010 串口通信程式")
    print("=" * 50)

    try:
        if not controller.connect():
            print("無法連接到設備，程式退出")
            return

        print("\n查詢當前 UNIT 值...")
        controller.query_unit()

        print("\n發送 AT+DISP=3 命令...")
        if not controller.enable_display_output():
            print("發送命令失敗")
            return
        if not controller.set_fps(args.fps):
            print("發送命令失敗")
            return

        print("\n開始監聽數據，按 Ctrl+C 停止...")
        monitor_thread = controller.start_monitoring()
        while monitor_thread.is_alive():
            monitor_thread.join(timeout=0.5)

    except KeyboardInterrupt:
        print("\n收到停止指令，正在結束串流...")
        controller.running = False
    except Exception as exc:
        print(f"程式執行錯誤：{exc}")
    finally:
        controller.running = False
        if monitor_thread:
            monitor_thread.join(timeout=2)
        controller.disconnect()
        cv2.destroyAllWindows()
        print("程式結束")


if __name__ == "__main__":
    main(sys.argv[1:])
