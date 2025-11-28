from __future__ import annotations

import time
from dataclasses import dataclass
from threading import Lock
from typing import Optional, Tuple


@dataclass
class PersonRangeReading:
    """
    Thread-safe snapshot describing the latest blind-person detection result.

    Attributes
    ----------
    distance_m : float
        Median distance from MaixSense (meters).
    bbox : Tuple[int, int, int, int]
        Bounding box (x, y, w, h) in sensor pixels for reference.
    area : float
        Contour area in pixels, useful for debugging / scaling.
    timestamp : float
        Epoch timestamp when the measurement was produced.
    """

    distance_m: float
    angle_deg: float
    bbox: Tuple[int, int, int, int]
    area: float
    timestamp: float


_lock = Lock()
_latest: Optional[PersonRangeReading] = None


def publish_person_range(reading: Optional[PersonRangeReading]) -> None:
    """
    Store the latest detection result so other modules can consume it.

    Passing ``None`` clears the current reading (e.g., no person detected).
    """

    global _latest
    with _lock:
        _latest = reading


def get_latest_person_range(max_age: Optional[float] = None) -> Optional[PersonRangeReading]:
    """
    Retrieve the freshest detection result.

    Parameters
    ----------
    max_age : Optional[float]
        Maximum age in seconds to consider valid. Older samples are discarded.
        When ``None``, no age filtering is applied.
    """

    with _lock:
        reading = _latest

    if reading is None:
        return None
    if max_age is not None and (time.time() - reading.timestamp) > max_age:
        return None
    return reading


def clear_person_range() -> None:
    """Convenience helper to drop any cached detection result."""

    publish_person_range(None)

