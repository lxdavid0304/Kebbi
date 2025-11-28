"""
Unified interface for ToF (MaixSense) human detection and RealSense D435 occupancy planning.
"""

from .system import KebbiVisionSystem, make_default_system

__all__ = ["KebbiVisionSystem", "make_default_system"]
