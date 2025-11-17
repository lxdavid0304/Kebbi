#!/usr/bin/env python3
"""
老師版架構：開啟 D435 即時顯示「翻正」的彩色點雲，按 q 儲存彩色/深度影像。
不做額外濾波或截斷，保留原始資料。
"""

from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
from realsense_depth import DepthCamera
from utils import createPointCloudO3D

# 解析度沿用老師設定
resolution_width, resolution_height = (640, 480)


def _desktop_path() -> Path:
    desktop = Path.home() / "Desktop"
    return desktop if desktop.exists() else Path.cwd()


def main():
    cam = DepthCamera(resolution_width, resolution_height)

    # 建立可互動的點雲視窗
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="PointCloud (flipped)", width=800, height=600)
    live_pcd = o3d.geometry.PointCloud()
    vis.add_geometry(live_pcd)

    out_dir = _desktop_path()

    try:
        while True:
            ret, depth_raw_frame, color_raw_frame = cam.get_raw_frame()
            if not ret:
                print("Unable to get a frame")
                continue

            # 彩色/深度原始影像（未處理）
            color_np = np.asanyarray(color_raw_frame.get_data())
            depth_np = np.asanyarray(depth_raw_frame.get_data())

            # 用老師的 createPointCloudO3D（含翻正 Y/Z）產生彩色點雲並顯示
            pcd = createPointCloudO3D(color_raw_frame, depth_raw_frame)
            live_pcd.points = pcd.points
            live_pcd.colors = pcd.colors
            vis.update_geometry(live_pcd)
            vis.poll_events()
            vis.update_renderer()

            # 顯示彩色影像
            cv2.imshow("Color", color_np)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                ts = time.strftime("%Y%m%d_%H%M%S")
                img_path = out_dir / f"frame_color_{ts}.png"
                depth_path = out_dir / f"frame_depth_{ts}.png"

                cv2.imwrite(str(img_path), color_np)
                # 深度直接存 16-bit PNG，保留原始深度值
                cv2.imwrite(str(depth_path), depth_np)

                print(f"Saved color -> {img_path}")
                print(f"Saved depth -> {depth_path}")
                break
    finally:
        vis.destroy_window()
        cam.release()


if __name__ == "__main__":
    main()
