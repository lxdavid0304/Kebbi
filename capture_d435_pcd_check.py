#!/usr/bin/env python3
"""
從 D435 讀取一幀並生成點雲，方便比對 capture_d435.py 前段點雲是否正確。
- 使用 d435capture.sensors 的 auto_start_realsense / frames_to_pointcloud_o3d
- 產生原始相機座標點雲，並套用老師的翻正（Y/Z 反轉）版本
- 按 Enter 抓拍；按 q 關閉視窗；會輸出 color PNG、depth PNG、raw PLY、flipped PLY

依賴：pyrealsense2、open3d、numpy、cv2、d435capture.sensors
"""

from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d

from d435capture.sensors import (
    auto_start_realsense,
    build_o3d_intrinsic_from_frame,
    frames_to_pointcloud_o3d,
)


def _desktop_path() -> Path:
    desktop = Path.home() / "Desktop"
    return desktop if desktop.exists() else Path.cwd()


def flip_pcd(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    """老師的翻正：Y/Z 取反，避免上下顛倒。"""
    pcd_copy = o3d.geometry.PointCloud(pcd)
    pcd_copy.transform([[1, 0, 0, 0],
                        [0, -1, 0, 0],
                        [0, 0, -1, 0],
                        [0, 0, 0, 1]])
    return pcd_copy


def main():
    pipeline, profile, align, depth_scale = auto_start_realsense()
    print("Stream started. 按 Enter 抓拍一幀並生成點雲...")
    input()

    frames = pipeline.wait_for_frames()
    if align is not None:
        frames = align.process(frames)
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    if not color_frame or not depth_frame:
        raise RuntimeError("無法取得 color/depth frame")

    color_np = np.asanyarray(color_frame.get_data())
    depth_np = np.asanyarray(depth_frame.get_data())

    # 生成點雲（相機座標），不降採樣，保留原始資訊
    intrinsic = build_o3d_intrinsic_from_frame(color_frame)
    pcd_raw = frames_to_pointcloud_o3d(
        color_np,
        depth_np,
        intrinsic,
        depth_trunc=5.0,   # 可視情況調整
        voxel_size=0.0,    # 不降採樣
        nb_neighbors=0,    # 不濾波
    )
    pcd_flip = flip_pcd(pcd_raw)

    out_dir = _desktop_path()
    ts = time.strftime("%Y%m%d_%H%M%S")
    img_path = out_dir / f"d435_color_{ts}.png"
    depth_path = out_dir / f"d435_depth_{ts}.png"
    raw_ply = out_dir / f"d435_raw_{ts}.ply"
    flip_ply = out_dir / f"d435_flip_{ts}.ply"

    cv2.imwrite(str(img_path), cv2.cvtColor(color_np, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(depth_path), depth_np)  # 16-bit depth
    o3d.io.write_point_cloud(str(raw_ply), pcd_raw)
    o3d.io.write_point_cloud(str(flip_ply), pcd_flip)

    print(f"Saved color -> {img_path}")
    print(f"Saved depth -> {depth_path}")
    print(f"Saved raw PLY -> {raw_ply}")
    print(f"Saved flipped PLY -> {flip_ply}")

    # 顯示翻正後點雲與彩色影像
    o3d.visualization.draw_geometries([pcd_flip], window_name="Flipped PointCloud")
    cv2.imshow("Color", color_np)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    pipeline.stop()


if __name__ == "__main__":
    main()
