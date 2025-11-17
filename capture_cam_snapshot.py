#!/usr/bin/env python3
"""
Capture a single frame from D435, save the color image and the raw camera-coordinate point cloud.
Files are written to the Desktop if available, otherwise to the current working directory.
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
    home = Path.home()
    desktop = home / "Desktop"
    return desktop if desktop.exists() else Path.cwd()


def main():
    pipeline, profile, align, depth_scale = auto_start_realsense()
    print("Stream started. 按 Enter 抓拍一張影像並存成相機座標點雲...")
    input()

    # 抓一幀
    frames = pipeline.wait_for_frames()
    if align is not None:
        frames = align.process(frames)
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    if not color_frame or not depth_frame:
        raise RuntimeError("無法取得 color/depth frame")

    intrinsic = build_o3d_intrinsic_from_frame(color_frame)
    color_np = np.asanyarray(color_frame.get_data())
    depth_np = np.asanyarray(depth_frame.get_data())

    # 相機座標的點雲，不做降採樣/濾波，保留原始資訊
    pcd_cam = frames_to_pointcloud_o3d(
        color_np,
        depth_np,
        intrinsic,
        voxel_size=0.0,        # 不降採樣
        depth_trunc=3.5,       # 視需求調整截斷距離
        nb_neighbors=0,        # 不做濾波
    )

    # 輸出路徑
    out_dir = _desktop_path()
    ts = time.strftime("%Y%m%d_%H%M%S")
    img_path = out_dir / f"cam_snapshot_{ts}.png"
    pcd_path = out_dir / f"cam_coords_{ts}.ply"

    # 存圖 + 點雲
    cv2.imwrite(str(img_path), cv2.cvtColor(color_np, cv2.COLOR_RGB2BGR))
    o3d.io.write_point_cloud(str(pcd_path), pcd_cam)

    print(f"Saved color image -> {img_path}")
    print(f"Saved camera-coordinate PCD -> {pcd_path}")

    pipeline.stop()


if __name__ == "__main__":
    main()
