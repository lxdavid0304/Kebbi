import pyrealsense2 as rs
import numpy as np


class DepthCamera:
    """
    簡易封裝：啟動 RealSense depth/color，對齊到 color，
    提供擷取對齊後的 frame（raw 或 numpy），並可讀取 depth_scale。
    """

    def __init__(self, resolution_width=640, resolution_height=480, fps=30, align_to=rs.stream.color):
        self.pipeline = rs.pipeline()
        config = rs.config()

        # 先解析裝置資訊
        wrapper = rs.pipeline_wrapper(self.pipeline)
        profile = config.resolve(wrapper)
        device = profile.get_device()
        depth_sensor = device.first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        # 對齊目標（預設 color）
        self.align = rs.align(align_to)

        # 啟用串流
        config.enable_stream(rs.stream.depth, resolution_width, resolution_height, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, resolution_width, resolution_height, rs.format.bgr8, fps)
        self.pipeline.start(config)

    def get_frame(self):
        """
        回傳對齊後的 depth/color 影像 (numpy)。
        """
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            return False, None, None
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        return True, depth_image, color_image

    def get_raw_frame(self):
        """
        回傳對齊後的 depth/color frame 物件 (pyrealsense2)。
        """
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_frame, color_frame

    def get_depth_scale(self):
        """
        depth 單位換算（通常 mm -> m，需乘以 depth_scale）。
        """
        return self.depth_scale

    def release(self):
        self.pipeline.stop()
