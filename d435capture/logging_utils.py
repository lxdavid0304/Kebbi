from __future__ import annotations

import platform
import sys
import traceback
from datetime import datetime
from importlib.metadata import PackageNotFoundError, version as pkg_version
from typing import Optional

from . import config as cfg

# Mirror legacy behaviour: print environment banner on import if enabled.
if cfg.PRINT_ENV_ON_IMPORT:
    py_ver = sys.version_info
    print(
        f"[env] Python {py_ver.major}.{py_ver.minor}.{py_ver.micro} | "
        "type-hints style = typing.Optional/Tuple/Dict (兼容 3.8+)."
    )


def _now() -> str:
    """Return formatted timestamp HH:MM:SS.mmm."""
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def log(step: str, msg: Optional[str] = None, *, flush: bool = True) -> None:
    """Central logging helper with optional step tag."""
    if msg is None:
        step, msg = "misc", step
    print(f"[{_now()}] [{step}] {msg}", flush=flush)


def log_ok(step: str = "misc") -> None:
    log(step, "✅ OK")


def log_warn(step: str, msg: str) -> None:
    log(step, f"⚠️ {msg}")


def log_fail(step: str, exc: BaseException) -> None:
    log(step, f"❌ FAIL: {type(exc).__name__}: {exc}")


def log_exc(step: str, exc: BaseException | None = None) -> None:
    """Print a rich traceback for easier debugging."""
    if exc is None:
        tb = traceback.format_exc()
        header = "EXCEPTION (no instance)"
    else:
        tb = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
        header = f"{type(exc).__name__}: {exc}"
    print(f"[{_now()}] [{step}] {header}\n{tb}", flush=True)


def _safe_log(step: str, msg: str) -> None:
    try:
        log(step, msg)
    except Exception:
        print(f"[{_now()}] [{step}] {msg}")


def _safe_log_ok(step: str) -> None:
    _safe_log(step, "✅ OK")


def _safe_log_exc(step: str, exc: BaseException | None = None) -> None:
    try:
        log_exc(step, exc)
    except Exception:
        if exc is None:
            print(f"[{_now()}] [{step}] EXCEPTION")
            print(traceback.format_exc())
        else:
            print(f"[{_now()}] [{step}] {type(exc).__name__}: {exc}")
            print("".join(traceback.format_exception(type(exc), exc, exc.__traceback__)))


def _pkg_ver_or_unknown(name: str) -> str:
    try:
        return pkg_version(name)
    except PackageNotFoundError:
        return "not-installed"
    except Exception:
        return "unknown"


def debug_env() -> None:
    """Print the runtime environment summary."""
    step = "debug_env"
    try:
        try:
            import cv2  # type: ignore

            cv_ver = getattr(cv2, "__version__", "unknown")
        except Exception:
            cv_ver = _pkg_ver_or_unknown("opencv-python")

        try:
            import open3d as o3d  # type: ignore

            o3d_ver = getattr(o3d, "__version__", "unknown")
            try:
                o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
            except Exception:
                pass
        except Exception:
            o3d_ver = _pkg_ver_or_unknown("open3d")

        rs_ver = _pkg_ver_or_unknown("pyrealsense2")

        log(step, "===== Environment =====")
        log(step, f"OS: {platform.platform()}")
        log(step, f"Python: {sys.version.split()[0]}")
        log(step, f"OpenCV: {cv_ver}")
        log(step, f"pyrealsense2: {rs_ver}")
        log(step, f"Open3D: {o3d_ver}")
        log(step, "=======================")
        log_ok(step)
    except Exception as exc:
        log_exc(step, exc)
        raise


def _dbg(msg: str):
    if cfg.DEBUG_CONVERSION:
        print(msg)


__all__ = [
    "log",
    "log_ok",
    "log_warn",
    "log_fail",
    "log_exc",
    "_safe_log",
    "_safe_log_ok",
    "_safe_log_exc",
    "_pkg_ver_or_unknown",
    "debug_env",
    "_dbg",
]
