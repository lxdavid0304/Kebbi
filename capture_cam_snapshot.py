#!/usr/bin/env python3
"""
按 Enter 抓拍一張 D435 影像，並以老師的相機座標轉點雲方法存成 PLY。
會同時輸出原始彩色圖 (PNG) 與相機座標點雲 (PLY) 到桌面（若桌面不存在則用當前目錄）。
"""

from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
import pyrealsense2 as rs

from d435capture.sensors import auto_start_realsense


# === 老師提供的相機座標點雲工具（保持原架構） ===
def depth2PointCloud(depth, rgb, depth_scale, clip_distance_max):
    """把深度影像轉成相機座標點雲，並附上 RGB。"""
    intrinsics = depth.profile.as_video_stream_profile().intrinsics
    depth = np.asanyarray(depth.get_data()) * depth_scale  # 1000 mm => 0.001 m
    rgb = np.asanyarray(rgb.get_data())
    rows, cols = depth.shape

    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    r = r.astype(float)
    c = c.astype(float)
    valid = (depth > 0) & (depth < clip_distance_max)  # 僅保留有效距離
    valid = np.ravel(valid)

    z = depth
    x = z * (c - intrinsics.ppx) / intrinsics.fx
    y = z * (r - intrinsics.ppy) / intrinsics.fy

    z = np.ravel(z)[valid]
    x = np.ravel(x)[valid]
    y = np.ravel(y)[valid]

    r_ch = np.ravel(rgb[:, :, 0])[valid]
    g_ch = np.ravel(rgb[:, :, 1])[valid]
    b_ch = np.ravel(rgb[:, :, 2])[valid]

    pointsxyzrgb = np.dstack((x, y, z, b_ch, g_ch, r_ch))
    pointsxyzrgb = pointsxyzrgb.reshape(-1, 6)
    return pointsxyzrgb


def get_intrinsic_matrix(frame, imwidth, imheight):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    out = o3d.camera.PinholeCameraIntrinsic(
        imwidth, imheight, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy
    )
    return out


def create_point_cloud_file2(vertices, filename):
    ply_header = """ply
	format ascii 1.0
	element vertex %(vert_num)d
	property float x
	property float y
	property float z
	property uchar red
	property uchar green
	property uchar blue
	end_header
	"""
    with open(filename, "w") as f:
        f.write(ply_header % dict(vert_num=len(vertices)))
        np.savetxt(f, vertices, "%f %f %f %d %d %d")


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

    color_np = np.asanyarray(color_frame.get_data())

    # 相機座標點雲（老師版本）
    clip_distance_max = 3.5
    vertices = depth2PointCloud(depth_frame, color_frame, depth_scale, clip_distance_max)

    # 輸出路徑
    out_dir = _desktop_path()
    ts = time.strftime("%Y%m%d_%H%M%S")
    img_path = out_dir / f"cam_snapshot_{ts}.png"
    pcd_path = out_dir / f"cam_coords_{ts}.ply"

    # 存影像 + 點雲
    cv2.imwrite(str(img_path), cv2.cvtColor(color_np, cv2.COLOR_RGB2BGR))
    create_point_cloud_file2(vertices, str(pcd_path))

    print(f"Saved color image -> {img_path}")
    print(f"Saved camera-coordinate PLY -> {pcd_path}")

    pipeline.stop()


if __name__ == "__main__":
    main()
