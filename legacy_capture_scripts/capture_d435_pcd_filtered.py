#!/usr/bin/env python3
"""
Capture one frame from D435, build a camera-frame point cloud, then apply a
light voxel downsample and gentle statistical outlier removal (so we keep
detail but stabilize the cloud). Saves raw/filtered/flip PLY plus color/depth.

Outputs:
- color PNG, depth PNG (16-bit)
- raw PLY (no filtering)
- filtered PLY (voxel + gentle outlier removal)
- flip_raw PLY, flip_filtered PLY (Y/Z flipped for easier viewing)

Deps: pyrealsense2, open3d, numpy, cv2, realsense_planner.sensors
"""

from __future__ import annotations

import time
from pathlib import Path

import _bootstrap  # noqa: F401
import cv2
import numpy as np
import open3d as o3d

from realsense_planner.sensors import (
    auto_start_realsense,
    build_o3d_intrinsic_from_frame,
    frames_to_pointcloud_o3d,
)


def _desktop_path() -> Path:
    desktop = Path.home() / "Desktop"
    return desktop if desktop.exists() else Path.cwd()


def flip_pcd(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    """Flip Y and Z for easier viewing (so up is +Y in the viewer)."""
    pcd_copy = o3d.geometry.PointCloud(pcd)
    pcd_copy.transform(
        [
            [1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [0, 0, 0, 1],
        ]
    )
    return pcd_copy


def main():
    pipeline, profile, align, depth_scale = auto_start_realsense()
    print("Stream started. Press Enter to capture one frame...")
    input()

    frames = pipeline.wait_for_frames()
    if align is not None:
        frames = align.process(frames)
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    if not color_frame or not depth_frame:
        raise RuntimeError("Missing color/depth frame")

    color_np = np.asanyarray(color_frame.get_data())
    depth_np = np.asanyarray(depth_frame.get_data())

    intrinsic = build_o3d_intrinsic_from_frame(color_frame)

    # Build camera-frame point cloud without filtering; truncate depth at 3.5 m
    pcd_raw = frames_to_pointcloud_o3d(
        color_np,
        depth_np,
        intrinsic,
        depth_trunc=3.5,
        voxel_size=0.0,  # 0 = no voxel in this pass
        nb_neighbors=0,  # 0 = no statistical outlier removal in this pass
    )

    # Light filter: small voxel + gentle statistical outlier removal
    if pcd_raw.has_points():
        pcd_filtered = pcd_raw.voxel_down_sample(voxel_size=0.01)
        if pcd_filtered.has_points():
            pcd_filtered, _ = pcd_filtered.remove_statistical_outlier(
                nb_neighbors=30,
                std_ratio=2.5,
            )
    else:
        pcd_filtered = o3d.geometry.PointCloud()

    # Flip for display
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

    # Show raw then filtered (flipped) one after the other
    if pcd_flip_raw.has_points():
        o3d.visualization.draw_geometries([pcd_flip_raw], window_name="Flipped Raw PCD")
    else:
        print("Raw point cloud is empty; cannot display.")

    if pcd_flip_filtered.has_points():
        o3d.visualization.draw_geometries([pcd_flip_filtered], window_name="Flipped Filtered PCD")
    else:
        print("Filtered point cloud is empty; cannot display.")

    cv2.imshow("Color", color_np)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    pipeline.stop()


if __name__ == "__main__":
    main()
