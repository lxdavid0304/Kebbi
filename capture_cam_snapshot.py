#!/usr/bin/env python3
"""
整合版：開啟 D435，對齊到 color，即時顯示翻正的彩色點雲與彩色影像。
按 q 儲存：
  - 彩色圖 (PNG)
  - 深度圖 (16-bit PNG)
  - 原始相機座標點雲 (PLY)
  - 翻正後點雲 (PLY，含彩色)

依賴：pyrealsense2、open3d、numpy、cv2
解析度：640x480，fps=30，未加任何濾波或截斷（保留深度>0 的點）。
"""

from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
import pyrealsense2 as rs


# ---------------------- RealSense 封裝 ----------------------
class DepthCamera:
    def __init__(self, w: int = 640, h: int = 480, fps: int = 30, align_to=rs.stream.color):
        self.pipeline = rs.pipeline()
        config = rs.config()
        wrapper = rs.pipeline_wrapper(self.pipeline)
        profile = config.resolve(wrapper)
        device = profile.get_device()
        depth_sensor = device.first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        self.align = rs.align(align_to)
        config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
        self.pipeline.start(config)

    def get_raw_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_frame, color_frame

    def release(self):
        self.pipeline.stop()


# ---------------------- utils ----------------------
def depth2PointCloud(depth_frame, color_frame, depth_scale, clip_distance_max=-1.0):
    """
    相機座標點雲 + RGB，不做濾波。clip_distance_max<0 表示不截斷，只丟 depth=0。
    """
    intr = depth_frame.profile.as_video_stream_profile().intrinsics
    depth_m = np.asanyarray(depth_frame.get_data()) * depth_scale
    color_np = np.asanyarray(color_frame.get_data())

    rows, cols = depth_m.shape
    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    r = r.astype(float)
    c = c.astype(float)

    if clip_distance_max > 0:
        valid = (depth_m > 0) & (depth_m < clip_distance_max)
    else:
        valid = (depth_m > 0)
    valid = np.ravel(valid)

    z = depth_m
    x = z * (c - intr.ppx) / intr.fx
    y = z * (r - intr.ppy) / intr.fy

    z = np.ravel(z)[valid]
    x = np.ravel(x)[valid]
    y = np.ravel(y)[valid]

    r_ch = np.ravel(color_np[:, :, 0])[valid]
    g_ch = np.ravel(color_np[:, :, 1])[valid]
    b_ch = np.ravel(color_np[:, :, 2])[valid]

    pointsxyzrgb = np.dstack((x, y, z, b_ch, g_ch, r_ch)).reshape(-1, 6)
    return pointsxyzrgb


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


def get_intrinsic_matrix(frame, imwidth, imheight):
    intr = frame.profile.as_video_stream_profile().intrinsics
    return o3d.camera.PinholeCameraIntrinsic(
        imwidth, imheight, intr.fx, intr.fy, intr.ppx, intr.ppy
    )


def createPointCloudO3D(color_frame, depth_frame):
    """
    用 Open3D 產生彩色點雲並翻正 (Y/Z 反轉)，保持彩色，不轉灰階。
    """
    color_np = np.asanyarray(color_frame.get_data())
    depth_np = np.asanyarray(depth_frame.get_data())
    h, w, _ = color_np.shape
    color_img = o3d.geometry.Image(color_np)
    depth_img = o3d.geometry.Image(depth_np)
    intrinsic = get_intrinsic_matrix(color_frame, w, h)

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_img, depth_img, convert_rgb_to_intensity=False
    )
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    # 翻正：Y/Z 取反
    pcd.transform([[1, 0, 0, 0],
                   [0, -1, 0, 0],
                   [0, 0, -1, 0],
                   [0, 0, 0, 1]])
    return pcd


def _desktop_path() -> Path:
    desktop = Path.home() / "Desktop"
    return desktop if desktop.exists() else Path.cwd()


# ---------------------- main ----------------------
def main():
    cam = DepthCamera()
    out_dir = _desktop_path()

    vis = o3d.visualization.Visualizer()
    vis.create_window("PointCloud (flipped)", width=800, height=600)
    live_pcd = o3d.geometry.PointCloud()
    vis.add_geometry(live_pcd)

    try:
        while True:
            ret, depth_raw, color_raw = cam.get_raw_frame()
            if not ret:
                print("Unable to get a frame")
                continue

            color_np = np.asanyarray(color_raw.get_data())
            depth_np = np.asanyarray(depth_raw.get_data())

            # 翻正彩色點雲即時顯示
            pcd = createPointCloudO3D(color_raw, depth_raw)
            live_pcd.points = pcd.points
            live_pcd.colors = pcd.colors
            vis.update_geometry(live_pcd)
            vis.poll_events()
            vis.update_renderer()

            cv2.imshow("Color", color_np)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                ts = time.strftime("%Y%m%d_%H%M%S")
                img_path = out_dir / f"frame_color_{ts}.png"
                depth_path = out_dir / f"frame_depth_{ts}.png"
                raw_ply_path = out_dir / f"cam_coords_raw_{ts}.ply"
                flip_ply_path = out_dir / f"cam_coords_flip_{ts}.ply"

                cv2.imwrite(str(img_path), color_np)
                cv2.imwrite(str(depth_path), depth_np)  # 16-bit depth
                vertices = depth2PointCloud(depth_raw, color_raw, cam.depth_scale, -1.0)
                create_point_cloud_file2(vertices, str(raw_ply_path))
                o3d.io.write_point_cloud(str(flip_ply_path), pcd)

                print(f"Saved color -> {img_path}")
                print(f"Saved depth -> {depth_path}")
                print(f"Saved raw PLY -> {raw_ply_path}")
                print(f"Saved flipped PLY -> {flip_ply_path}")
                break
    finally:
        vis.destroy_window()
        cam.release()


if __name__ == "__main__":
    main()
