from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class PersonDetectionParams:
    """Configuration knobs for person detection on the depth map."""

    min_distance_mm: float = 500.0
    max_distance_mm: float = 1500.0
    min_area: int = 500
    aspect_ratio_range: Tuple[float, float] = (0.8, 2.5)
    min_solidity: float = 0.7
    smoothing_kernel: int = 5
    max_tracking_misses: int = 5


@dataclass
class PersonDetection:
    """Holds a single detection result in image coordinates."""

    x: int
    y: int
    w: int
    h: int
    cx: int
    cy: int
    median_distance: float

    @property
    def area(self) -> int:
        return self.w * self.h


@dataclass
class PersonDetector:
    """Encapsulates contour-based person detection and lightweight temporal tracking."""

    params: PersonDetectionParams = field(default_factory=PersonDetectionParams)
    _last_detections: List[PersonDetection] = field(default_factory=list, init=False)
    _missed_detection_frames: int = field(default=0, init=False)

    def detect(self, distance_image: np.ndarray) -> Optional[List[PersonDetection]]:
        params = self.params
        kernel_size = max(1, int(params.smoothing_kernel))
        if kernel_size % 2 == 0:
            kernel_size += 1

        smoothed = cv2.GaussianBlur(distance_image, (kernel_size, kernel_size), 0)
        mask = cv2.inRange(smoothed, params.min_distance_mm, params.max_distance_mm)

        morph_kernel = np.ones((3, 3), np.uint8)
        mask_cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, morph_kernel, iterations=2)
        mask_cleaned = cv2.morphologyEx(mask_cleaned, cv2.MORPH_CLOSE, morph_kernel, iterations=2)

        contours, _ = cv2.findContours(mask_cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections: List[PersonDetection] = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < params.min_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w == 0 or h == 0:
                continue

            aspect_ratio = float(h) / w
            min_ar, max_ar = params.aspect_ratio_range
            if not (min_ar <= aspect_ratio <= max_ar):
                continue

            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            if hull_area == 0:
                continue
            solidity = float(area) / hull_area
            if solidity < params.min_solidity:
                continue

            person_roi = distance_image[y : y + h, x : x + w]
            roi_mask = mask_cleaned[y : y + h, x : x + w] > 0
            valid_distances = person_roi[roi_mask]
            if valid_distances.size == 0:
                continue

            detections.append(
                PersonDetection(
                    x=x,
                    y=y,
                    w=w,
                    h=h,
                    cx=x + w // 2,
                    cy=y + h // 2,
                    median_distance=float(np.median(valid_distances)),
                )
            )

        if detections:
            detections.sort(key=lambda det: det.area, reverse=True)
            self._last_detections = detections
            self._missed_detection_frames = 0
            return detections

        self._missed_detection_frames += 1
        if (
            self._last_detections
            and self._missed_detection_frames <= params.max_tracking_misses
        ):
            return self._last_detections

        self._last_detections = []
        return None
