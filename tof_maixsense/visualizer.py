from __future__ import annotations

from typing import List, Optional

import cv2
import numpy as np

from .detection import PersonDetection, PersonDetector


class ImageVisualizer:
    """Handles OpenCV visualization and overlays detection results."""

    def __init__(self, detector: PersonDetector, scale_factor: int = 4):
        self.detector = detector
        self.scale_factor = scale_factor

    def display(self, raw_image: np.ndarray, distance_image: np.ndarray):
        raw_display = cv2.normalize(raw_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        distance_processed, valid_mask = self._prepare_distance(distance_image)

        raw_resized = cv2.resize(raw_display, (400, 400), interpolation=cv2.INTER_NEAREST)
        distance_resized = cv2.resize(distance_processed, (400, 400), interpolation=cv2.INTER_NEAREST)

        detections = self.detector.detect(distance_image)
        raw_resized_bgr = cv2.cvtColor(raw_resized, cv2.COLOR_GRAY2BGR)
        if detections:
            self._draw_detections(distance_resized, raw_resized_bgr, detections)

        combined = self._compose_view(raw_resized_bgr, distance_resized)
        cv2.imshow("MaixSense ToF Camera - Raw & Distance", combined)

        min_dist = np.min(distance_image[distance_image > 0]) if np.any(distance_image > 0) else 0
        max_dist = np.max(distance_image)
        mean_dist = np.mean(distance_image[distance_image > 0]) if np.any(distance_image > 0) else 0
        valid_pixels = int(np.sum(valid_mask))
        total_pixels = int(distance_image.size)
        print(
            f"距離統計 - 最小值: {min_dist:.1f}mm, 最大值: {max_dist:.1f}mm, "
            f"平均值: {mean_dist:.1f}mm, 有效像素: {valid_pixels}/{total_pixels}"
        )
        cv2.waitKey(1)
        return detections

    def _prepare_distance(self, distance_image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        valid_mask = (distance_image > 0) & (distance_image < 2000)
        processed_distance = distance_image.copy()
        processed_distance[~valid_mask] = 0

        if np.any(valid_mask):
            max_valid_dist = np.max(processed_distance[valid_mask])
            inverted_distance = np.zeros_like(processed_distance)
            inverted_distance[valid_mask] = max_valid_dist - processed_distance[valid_mask]
            distance_normalized = cv2.normalize(
                inverted_distance, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
            )
        else:
            distance_normalized = np.zeros_like(processed_distance, dtype=np.uint8)

        distance_colored = cv2.applyColorMap(distance_normalized, cv2.COLORMAP_HOT)
        distance_colored[~valid_mask] = [0, 0, 0]
        return distance_colored, valid_mask

    def _draw_detections(
        self,
        distance_resized: np.ndarray,
        raw_resized_bgr: np.ndarray,
        detections: List[PersonDetection],
    ):
        colors = [(0, 255, 0), (0, 165, 255), (255, 0, 0)]
        for idx, det in enumerate(detections[:3]):
            color = colors[idx % len(colors)]
            x_scaled = det.x * self.scale_factor
            y_scaled = det.y * self.scale_factor
            w_scaled = det.w * self.scale_factor
            h_scaled = det.h * self.scale_factor
            cx_scaled = det.cx * self.scale_factor
            cy_scaled = det.cy * self.scale_factor

            cv2.rectangle(
                distance_resized,
                (x_scaled, y_scaled),
                (x_scaled + w_scaled, y_scaled + h_scaled),
                color,
                2,
            )
            cv2.circle(distance_resized, (cx_scaled, cy_scaled), 5, (0, 0, 255), -1)
            label = f"P{idx+1}: {det.median_distance:.0f} mm"
            text_y = max(20, y_scaled - 10)
            cv2.putText(
                distance_resized,
                label,
                (x_scaled, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )

            cv2.rectangle(
                raw_resized_bgr,
                (x_scaled, y_scaled),
                (x_scaled + w_scaled, y_scaled + h_scaled),
                color,
                2,
            )
            cv2.putText(
                raw_resized_bgr,
                label,
                (x_scaled, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )

    def _compose_view(self, raw_resized_bgr: np.ndarray, distance_resized: np.ndarray) -> np.ndarray:
        raw_with_title = np.zeros((430, 400, 3), dtype=np.uint8)
        raw_with_title[30:430, :] = raw_resized_bgr
        cv2.putText(
            raw_with_title,
            "Raw Image (Flipped)",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

        distance_with_title = np.zeros((430, 400, 3), dtype=np.uint8)
        distance_with_title[30:430, :] = distance_resized
        cv2.putText(
            distance_with_title,
            "Distance Map (Person Detection)",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
        return np.hstack([raw_with_title, distance_with_title])
