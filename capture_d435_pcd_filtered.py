#!/usr/bin/env python3
"""
抓一幀 D435，生成相機座標點雲，並套用與 capture_d435/cli 相同的點雲層降採樣/統計濾波，
同時輸出原始/濾波後/翻正版本，方便比較效果。

輸出：
- color PNG, depth PNG (16-bit)
- raw PLY (未降採樣/濾波)
- filtered PLY (降採樣+statistical outlier removal)
- flip_raw PLY, flip_filtered PLY (Y/Z 反轉)

依賴：pyrealsense2, open3d, numpy, cv2, d435capture.sensors
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
    """Y/Z 反轉，避免上下顛倒。"""
    pcd_copy = o3d.geometry.PointCloud(pcd)
    pcd_copy.transform([[1, 0, 0, 0],
                        [0, -1, 0, 0],
                        [0, 0, -1, 0],
                        [0, 0, 0, 1]])
    return pcd_copy


def main():
    pipeline, profile, align, depth_scale = auto_start_realsense()
    print("Stream started. 按 Enter 抓拍並生成點雲（含濾波版）...")
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

    intrinsic = build_o3d_intrinsic_from_frame(color_frame)

    # 原始相機座標點雲（不降採樣、不濾波），截斷 3.5 m
    pcd_raw = frames_to_pointcloud_o3d(
        color_np,
        depth_np,
        intrinsic,
        depth_trunc=3.5,   # 視需求調整截斷（現改為 3.5 m）
        voxel_size=0.0,    # 0 代表不做 voxel downsample
        nb_neighbors=0,    # 0 代表不做統計濾波
    )

    # 套用 capture_d435/cli 的降採樣 + 統計濾波
    # 依據 sensors.frames_to_pointcloud_o3d 的參數：vs=voxel_size, nb_neighbors 統計濾波鄰居，std_ratio=2.0
    if pcd_raw.has_points():
        pcd_filtered = pcd_raw.voxel_down_sample(voxel_size=0.02)
        if pcd_filtered.has_points():
            pcd_filtered, _ = pcd_filtered.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    else:
        pcd_filtered = o3d.geometry.PointCloud()

    # 翻正
    pcd_flip_raw = flip_pcd(pcd_raw)
    pcd_flip_filtered = flip_pcd(pcd_filtered)

    out_dir = _desktop_path()
    ts = time.strftime("%Y%m%d_%H%M%S")
    img_path = out_dir / f"d435_color_{ts}.png"
    depth_path = out_dir / f"d435_depth_{ts}.png"
    raw_ply = out_dir / f"d435_raw_{ts}.ply"
    filt_ply = out_dir / f"d435_filtered_{ts}.ply"
    flip_raw_ply = out_dir / f"d435_flip_raw_{ts}.ply"
    flip_filt_ply = out_dir / f"d435_flip_filtered_{ts}.ply"

    cv2.imwrite(str(img_path), cv2.cvtColor(color_np, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(depth_path), depth_np)  # 16-bit depth
    o3d.io.write_point_cloud(str(raw_ply), pcd_raw)
    o3d.io.write_point_cloud(str(filt_ply), pcd_filtered)
    o3d.io.write_point_cloud(str(flip_raw_ply), pcd_flip_raw)
    o3d.io.write_point_cloud(str(flip_filt_ply), pcd_flip_filtered)

    print(f"Saved color -> {img_path}")
    print(f"Saved depth -> {depth_path}")
    print(f"Saved raw PLY -> {raw_ply}")
    print(f"Saved filtered PLY -> {filt_ply}")
    print(f"Saved flip raw PLY -> {flip_raw_ply}")
    print(f"Saved flip filtered PLY -> {flip_filt_ply}")

    # 顯示：原始與濾波後翻正點雲（兩個視窗依序顯示）
    if pcd_flip_raw.has_points():
        o3d.visualization.draw_geometries([pcd_flip_raw], window_name="Flipped Raw PCD")
    else:
        print("原始點雲為空，無法顯示 Raw 視窗。")

    if pcd_flip_filtered.has_points():
        o3d.visualization.draw_geometries([pcd_flip_filtered], window_name="Flipped Filtered PCD")
    else:
        print("濾波點雲為空，無法顯示 Filtered 視窗。")

    cv2.imshow("Color", color_np)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    pipeline.stop()


if __name__ == "__main__":
    main()
