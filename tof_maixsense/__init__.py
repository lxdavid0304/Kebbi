"""MaixSense package consolidating detection, visualization, and state helpers."""

from __future__ import annotations

from importlib import import_module
from typing import Any

from . import state as state_module

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
    "append_detection",
    "append_voice_event",
    "mark_voice_processed",
    "read_pending_events",
    "update_odom",
    "update_meta",
    "load_state",
    "save_state",
    "state",
]

_LAZY_ATTRS = {
    "MaixSenseController": ("tof_maixsense.controller", "MaixSenseController"),
    "PersonDetection": ("tof_maixsense.detection", "PersonDetection"),
    "PersonDetectionParams": ("tof_maixsense.detection", "PersonDetectionParams"),
    "PersonDetector": ("tof_maixsense.detection", "PersonDetector"),
    "ImageVisualizer": ("tof_maixsense.visualizer", "ImageVisualizer"),
    "PacketParser": ("tof_maixsense.packets", "PacketParser"),
    "PacketProcessingResult": ("tof_maixsense.packets", "PacketProcessingResult"),
}

state = state_module
PersonRangeReading = state_module.PersonRangeReading
clear_person_range = state_module.clear_person_range
get_latest_person_range = state_module.get_latest_person_range
publish_person_range = state_module.publish_person_range
append_detection = state_module.append_detection
append_voice_event = state_module.append_voice_event
mark_voice_processed = state_module.mark_voice_processed
read_pending_events = state_module.read_pending_events
update_odom = state_module.update_odom
update_meta = state_module.update_meta
load_state = state_module.load_state
save_state = state_module.save_state


def __getattr__(name: str) -> Any:
    if name in _LAZY_ATTRS:
        module_name, attr = _LAZY_ATTRS[name]
        module = import_module(module_name)
        value = getattr(module, attr)
        globals()[name] = value
        return value
    raise AttributeError(f"module 'tof_maixsense' has no attribute '{name}'")


def __dir__() -> list[str]:
    return sorted(set(__all__))
