import importlib

import pytest


@pytest.fixture()
def state_module(tmp_path, monkeypatch):
    state_path = tmp_path / "state.json"
    monkeypatch.setenv("KEBBI_STATE_PATH", str(state_path))
    import tof_maixsense.state as state

    importlib.reload(state)
    yield state


def test_append_detection_keeps_fixed_window(state_module):
    for idx in range(6):
        state_module.append_detection(distance_m=1.0 + idx, angle_deg=idx)

    snapshot = state_module.load_state()
    detections = snapshot["detections"]
    assert len(detections) == state_module.MAX_DETECTIONS
    assert detections[-1]["angle"] == 5
    assert detections[0]["angle"] == 1


def test_voice_events_queue_and_mark(state_module):
    ev1 = state_module.append_voice_event("向前移動")
    ev2 = state_module.append_voice_event("向後移動")

    snapshot = state_module.read_pending_events()
    assert [ev["event_id"] for ev in snapshot["voice_events"]] == [ev1["event_id"], ev2["event_id"]]

    state_module.mark_voice_processed(ev1["event_id"])
    snapshot = state_module.read_pending_events()
    assert [ev["event_id"] for ev in snapshot["voice_events"]] == [ev2["event_id"]]


def test_update_odom_and_meta(state_module):
    state_module.update_odom(position={"x": 1.2, "y": -0.5}, orientation={"yaw": 45.0})
    state_module.update_meta(last_detection="none", tcp_status="connected")
    snapshot = state_module.load_state()
    assert snapshot["odom"]["position"]["x"] == pytest.approx(1.2)
    assert snapshot["odom"]["position"]["y"] == pytest.approx(-0.5)
    assert snapshot["odom"]["orientation"]["yaw"] == pytest.approx(45.0)
    assert snapshot["meta"]["last_detection"] == "none"
    assert snapshot["meta"]["tcp_status"] == "connected"
