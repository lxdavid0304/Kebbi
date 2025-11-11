from __future__ import annotations

import threading
from typing import Callable, Iterable, List, Optional, Sequence

from Human_Detection_Depth_Camera import main as tof_cli_main
from d435capture.cli import main as realsense_cli_main


SubsystemEntry = Callable[[], None]


def make_tof_entry(argv: Optional[Sequence[str]] = None) -> SubsystemEntry:
    args = list(argv) if argv is not None else []

    def _runner():
        tof_cli_main(list(args))

    return _runner


def make_realsense_entry(argv: Optional[Sequence[str]] = None) -> SubsystemEntry:
    args = list(argv) if argv is not None else []

    def _runner():
        realsense_cli_main(list(args))

    return _runner


class KebbiVisionSystem:
    """
    Orchestrates both ToF human detection and RealSense occupancy planning subsystems.
    """

    def __init__(
        self,
        tof_entry: Optional[SubsystemEntry] = None,
        realsense_entry: Optional[SubsystemEntry] = None,
    ):
        self._tof_entry = tof_entry
        self._realsense_entry = realsense_entry
        self._threads: List[threading.Thread] = []

    def configure_tof(self, entry: SubsystemEntry) -> None:
        self._tof_entry = entry

    def configure_realsense(self, entry: SubsystemEntry) -> None:
        self._realsense_entry = entry

    def run(
        self,
        *,
        enable_tof: bool = True,
        enable_realsense: bool = True,
        join: bool = True,
    ) -> List[threading.Thread]:
        self._threads = []
        if enable_tof and self._tof_entry:
            self._threads.append(self._start_thread("tof-subsystem", self._tof_entry))
        if enable_realsense and self._realsense_entry:
            self._threads.append(
                self._start_thread("realsense-subsystem", self._realsense_entry)
            )

        if join:
            for thread in self._threads:
                thread.join()

        return list(self._threads)

    def _start_thread(self, name: str, target: SubsystemEntry) -> threading.Thread:
        thread = threading.Thread(target=target, name=name, daemon=False)
        thread.start()
        return thread


def make_default_system(
    *,
    tof_args: Optional[Sequence[str]] = None,
    realsense_args: Optional[Sequence[str]] = None,
) -> KebbiVisionSystem:
    system = KebbiVisionSystem(
        tof_entry=make_tof_entry(tof_args),
        realsense_entry=make_realsense_entry(realsense_args),
    )
    return system
