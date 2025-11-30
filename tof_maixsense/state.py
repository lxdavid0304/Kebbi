"""Shared state manager for MaixSense detection and RealSense planning."""

from __future__ import annotations

import copy
import fcntl
import json
import os
import time
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from threading import Lock
from typing import Any, Callable, Dict, List, Optional, Tuple


# ---------------------------------------------------------------------------
# Backward compatibility helpers (used by older modules/tests).
# ---------------------------------------------------------------------------


@dataclass
class PersonRangeReading:
    """Legacy structure describing the latest blind-person detection result."""

    distance_m: float
    angle_deg: float
    bbox: Tuple[int, int, int, int]
    area: float
    timestamp: float


_legacy_lock = Lock()
_legacy_latest: Optional[PersonRangeReading] = None


def publish_person_range(reading: Optional[PersonRangeReading]) -> None:
    """Store the latest detection result for legacy consumers."""

    global _legacy_latest
    with _legacy_lock:
        _legacy_latest = reading


def get_latest_person_range(max_age: Optional[float] = None) -> Optional[PersonRangeReading]:
    """Return the freshest legacy detection result if still valid."""

    with _legacy_lock:
        reading = _legacy_latest

    if reading is None:
        return None
    if max_age is not None and (time.time() - reading.timestamp) > max_age:
        return None
    return reading


def clear_person_range() -> None:
    """Clear any cached legacy detection result."""

    publish_person_range(None)


# ---------------------------------------------------------------------------
# File-backed shared state helpers.
# ---------------------------------------------------------------------------


STATE_PATH = Path(os.environ.get("KEBBI_STATE_PATH", "/tmp/kebbi_state.json"))
STATE_VERSION = 1
MAX_DETECTIONS = 5
MAX_VOICE_EVENTS = 32


def _default_state() -> Dict[str, Any]:
    return {
        "version": STATE_VERSION,
        "detections": [],
        "voice_events": [],
        "odom": {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "timestamp": 0.0,
        },
        "meta": {
            "last_detection_status": "unknown",
            "last_detection": "none",
            "last_detection_timestamp": 0.0,
            "last_event_id_sent": 0,
            "next_event_id": 1,
            "tcp_status": "disconnected",
            "error_flags": [],
            "last_seen_pose": None,
            "updated_at": 0.0,
        },
    }


def _ensure_state_file() -> None:
    STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
    if not STATE_PATH.exists():
        STATE_PATH.write_text(json.dumps(_default_state(), ensure_ascii=False, indent=2), encoding="utf-8")


def _merge_defaults(state: Dict[str, Any]) -> Dict[str, Any]:
    baseline = copy.deepcopy(_default_state())
    for key, value in state.items():
        if isinstance(value, dict):
            existing = baseline.get(key)
            if isinstance(existing, dict):
                existing.update(value)
            else:
                baseline[key] = value
        else:
            baseline[key] = value
    if "meta" not in baseline or not isinstance(baseline["meta"], dict):
        baseline["meta"] = _default_state()["meta"]
    baseline["meta"].setdefault("next_event_id", 1)
    baseline.setdefault("detections", [])
    baseline.setdefault("voice_events", [])
    return baseline


@contextmanager
def _locked_state(lock_type: int):
    _ensure_state_file()
    with STATE_PATH.open("r+", encoding="utf-8") as fh:
        fcntl.flock(fh.fileno(), lock_type)
        fh.seek(0)
        raw = fh.read().strip()
        if not raw:
            state = _default_state()
        else:
            try:
                state = json.loads(raw)
            except json.JSONDecodeError:
                state = _default_state()
        state = _merge_defaults(state)
        yield state, fh
        fcntl.flock(fh.fileno(), fcntl.LOCK_UN)


def load_state() -> Dict[str, Any]:
    """Load a snapshot of the shared state (read-only)."""

    with _locked_state(fcntl.LOCK_SH) as (state, _):
        return copy.deepcopy(state)


def save_state(new_state: Dict[str, Any]) -> None:
    """Overwrite the shared state with ``new_state`` atomically."""

    normalized = _merge_defaults(new_state)

    with _locked_state(fcntl.LOCK_EX) as (_, fh):
        fh.seek(0)
        json.dump(normalized, fh, ensure_ascii=False, indent=2)
        fh.truncate()
        fh.flush()
        os.fsync(fh.fileno())


def _mutate_state(mutator: Callable[[Dict[str, Any]], Any]) -> Any:
    with _locked_state(fcntl.LOCK_EX) as (state, fh):
        result = mutator(state)
        fh.seek(0)
        json.dump(state, fh, ensure_ascii=False, indent=2)
        fh.truncate()
        fh.flush()
        os.fsync(fh.fileno())
        return result


def _next_event_id(state: Dict[str, Any]) -> int:
    event_id = int(state["meta"].get("next_event_id", 1))
    state["meta"]["next_event_id"] = event_id + 1
    return event_id


def append_detection(
    distance_m: float,
    angle_deg: float,
    *,
    tts_text: Optional[str] = None,
    status: str = "unknown",
    timestamp: Optional[float] = None,
) -> Dict[str, Any]:
    """Append a detection event and keep the queue length bounded."""

    ts = timestamp or time.time()

    def _mutator(state: Dict[str, Any]) -> Dict[str, Any]:
        event = {
            "event_id": _next_event_id(state),
            "distance": float(distance_m),
            "angle": float(angle_deg),
            "tts_text": tts_text,
            "timestamp": ts,
            "status": status,
        }
        detections: List[Dict[str, Any]] = state.setdefault("detections", [])
        detections.append(event)
        if len(detections) > MAX_DETECTIONS:
            del detections[0 : len(detections) - MAX_DETECTIONS]
        meta = state.setdefault("meta", {})
        meta["last_detection_status"] = status
        meta["last_detection"] = status
        meta["last_detection_timestamp"] = ts
        meta["last_seen_pose"] = {
            "distance": event["distance"],
            "angle": event["angle"],
            "timestamp": ts,
        }
        meta["updated_at"] = ts
        return event

    return _mutate_state(_mutator)


def append_voice_event(
    command: str,
    *,
    timestamp: Optional[float] = None,
) -> Dict[str, Any]:
    """Queue a voice command for the RealSense/TCP bridge to deliver."""

    ts = timestamp or time.time()

    def _mutator(state: Dict[str, Any]) -> Dict[str, Any]:
        event = {
            "event_id": _next_event_id(state),
            "command": command,
            "timestamp": ts,
            "processed": False,
        }
        events: List[Dict[str, Any]] = state.setdefault("voice_events", [])
        events.append(event)
        if len(events) > MAX_VOICE_EVENTS:
            del events[0 : len(events) - MAX_VOICE_EVENTS]
        return event

    return _mutate_state(_mutator)


def mark_voice_processed(event_id: int, *, remove: bool = True) -> bool:
    """Mark a queued voice event as delivered (optionally removing it)."""

    def _mutator(state: Dict[str, Any]) -> bool:
        events: List[Dict[str, Any]] = state.setdefault("voice_events", [])
        found = False
        if remove:
            remaining: List[Dict[str, Any]] = []
            for event in events:
                if event.get("event_id") == event_id:
                    event["processed"] = True
                    state.setdefault("meta", {})["last_event_id_sent"] = event_id
                    found = True
                else:
                    remaining.append(event)
            state["voice_events"] = remaining
        else:
            for event in events:
                if event.get("event_id") == event_id:
                    event["processed"] = True
                    state.setdefault("meta", {})["last_event_id_sent"] = event_id
                    found = True
        return found

    return _mutate_state(_mutator)


def update_odom(
    *,
    position: Optional[Dict[str, float]] = None,
    orientation: Optional[Dict[str, float]] = None,
    timestamp: Optional[float] = None,
) -> Dict[str, Any]:
    """Update the odometry block with the latest reading."""

    ts = timestamp or time.time()

    def _mutator(state: Dict[str, Any]) -> Dict[str, Any]:
        odom = state.setdefault("odom", {})
        pos = odom.setdefault("position", {"x": 0.0, "y": 0.0, "z": 0.0})
        if position:
            pos.update({k: float(v) for k, v in position.items()})
        orient = odom.setdefault("orientation", {"roll": 0.0, "pitch": 0.0, "yaw": 0.0})
        if orientation:
            orient.update({k: float(v) for k, v in orientation.items()})
        odom["timestamp"] = ts
        return copy.deepcopy(odom)

    return _mutate_state(_mutator)


def read_pending_events() -> Dict[str, Any]:
    """Return detections, meta info, and unprocessed voice events."""

    snapshot = load_state()
    pending_voice = [ev for ev in snapshot.get("voice_events", []) if not ev.get("processed")]
    snapshot["voice_events"] = pending_voice
    detections = snapshot.get("detections", [])
    if len(detections) > MAX_DETECTIONS:
        snapshot["detections"] = detections[-MAX_DETECTIONS:]
    return snapshot


def update_meta(**kwargs: Any) -> Dict[str, Any]:
    """Update arbitrary meta fields inside the shared state."""

    def _mutator(state: Dict[str, Any]) -> Dict[str, Any]:
        meta = state.setdefault("meta", {})
        meta.update(kwargs)
        meta["updated_at"] = time.time()
        return copy.deepcopy(meta)

    return _mutate_state(_mutator)
