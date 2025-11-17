#!/usr/bin/env python3
"""
以老師的流程即時開啟 D435 串流，顯示翻正的彩色點雲與彩色影像。
- 使用老師版的 createPointCloudO3D（含 Y/Z 翻正、不轉灰階）
- 解析度 640x480、30fps，對齊到 color
- 按 q 離開

依賴：pyrealsense2、open3d、numpy、cv2
"""

from __future__ import annotations

import cv2
import numpy as np
import open3d as o3d
import pyrealsense2 as rs


# ---------------------- DepthCamera ----------------------
class DepthCamera:
    def __init__(self, w: int = 640, h: int = 480, fps: int = 30, align_to=rs.stream.color):
        self.pipeline = rs.pipeline()
        config = rs.config()
        wrapper = rs.pipeline_wrapper(self.pipeline)
        profile = config.resolve(wrapper)
        device = profile.get_device()
        depth_sensor = device.first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        # 對齊到指定串流
        self.align = rs.align(align_to)
        # 啟用串流
        config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
        self.pipeline.start(config)
        # 強制開啟投射器／雷射
        try:
            if depth_sensor.supports(rs.option.emitter_enabled):
                depth_sensor.set_option(rs.option.emitter_enabled, 1)
            if depth_sensor.supports(rs.option.laser_power):
                rng = depth_sensor.get_option_range(rs.option.laser_power)
                depth_sensor.set_option(rs.option.laser_power, rng.max)
        except Exception:
            pass

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


# ---------------------- utils (老師版) ----------------------
def get_intrinsic_matrix(frame, imwidth, imheight):
    intr = frame.profile.as_video_stream_profile().intrinsics
    return o3d.camera.PinholeCameraIntrinsic(
        imwidth, imheight, intr.fx, intr.fy, intr.ppx, intr.ppy
    )


def createPointCloudO3D(color_frame, depth_frame):
    """
    用 Open3D 建立彩色點雲並翻正 (Y/Z 反轉)，保持彩色。
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


# ---------------------- main ----------------------
def main():
    cam = DepthCamera()

    # Open3D 視窗
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

            # 檢查深度是否有效
            print("valid depth:", (depth_np > 0).sum(), "max depth:", depth_np.max())

            # 更新點雲
            pcd = createPointCloudO3D(color_raw, depth_raw)
            live_pcd.points = pcd.points
            live_pcd.colors = pcd.colors
            vis.update_geometry(live_pcd)
            vis.poll_events()
            vis.update_renderer()

            # 顯示彩色影像
            cv2.imshow("Color", color_np)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    finally:
        vis.destroy_window()
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
