"""
MaixSense package consolidating controller, detection, visualization, and packet parsing.
"""

from .controller import MaixSenseController
from .detection import PersonDetection, PersonDetectionParams, PersonDetector
from .visualizer import ImageVisualizer
from .packets import PacketParser, PacketProcessingResult
from .state import (
    PersonRangeReading,
    clear_person_range,
    get_latest_person_range,
    publish_person_range,
)

__all__ = [
    "MaixSenseController",
    "PersonDetection",
    "PersonDetectionParams",
    "PersonDetector",
    "ImageVisualizer",
    "PacketParser",
    "PacketProcessingResult",
    "PersonRangeReading",
    "get_latest_person_range",
    "publish_person_range",
    "clear_person_range",
]
