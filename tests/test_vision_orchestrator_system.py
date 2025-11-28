import time

from vision_orchestrator.system import KebbiVisionSystem


def test_system_runs_both_subsystems():
    calls = []

    def tof_stub():
        calls.append("tof")

    def rs_stub():
        calls.append("rs")

    system = KebbiVisionSystem(tof_entry=tof_stub, realsense_entry=rs_stub)
    system.run(join=True)

    assert set(calls) == {"tof", "rs"}


def test_system_respects_enable_flags():
    calls = []

    def tof_stub():
        calls.append("tof")

    system = KebbiVisionSystem(tof_entry=tof_stub, realsense_entry=None)
    system.run(enable_tof=True, enable_realsense=False, join=True)

    assert calls == ["tof"]
