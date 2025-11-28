#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CLI that launches both MaixSense ToF detection and RealSense occupancy planning subsystems.
"""

from __future__ import annotations

import argparse
from typing import List, Optional, Sequence

from realsense_planner import config as rs_cfg
from vision_orchestrator.system import KebbiVisionSystem, make_default_system


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Unified launcher for MaixSense ToF detection and RealSense D435 planning."
    )
    parser.add_argument("--disable-tof", action="store_true", help="Skip ToF subsystem")
    parser.add_argument(
        "--disable-realsense", action="store_true", help="Skip RealSense subsystem"
    )

    tof = parser.add_argument_group("ToF (MaixSense) options")
    tof.add_argument("--tof-port", default="/dev/ttyUSB0", help="ToF serial port")
    tof.add_argument("--tof-baudrate", type=int, default=57600, help="ToF baudrate")
    tof.add_argument("--tof-fps", type=int, default=4, help="ToF output FPS")
    tof.add_argument(
        "--tof-min-distance",
        type=float,
        default=500.0,
        help="Minimum detection distance (mm)",
    )
    tof.add_argument(
        "--tof-max-distance",
        type=float,
        default=1500.0,
        help="Maximum detection distance (mm)",
    )
    tof.add_argument(
        "--tof-min-area", type=int, default=500, help="Minimum person contour area"
    )
    tof.add_argument("--tof-verbose", action="store_true", help="Enable verbose logging")

    rs = parser.add_argument_group("RealSense (D435) options")
    rs.add_argument("--rs-tcp-ip", default=rs_cfg.ROBOT_TCP_IP, help="Robot TCP IP")
    rs.add_argument("--rs-tcp-port", type=int, default=rs_cfg.ROBOT_TCP_PORT, help="Robot TCP port")
    rs.add_argument("--rs-no-tcp", action="store_true", help="Disable TCP commands")
    rs.add_argument("--rs-cam-pitch", type=float, default=40.0, help="Camera pitch (deg)")
    rs.add_argument("--rs-cam-height", type=float, default=0.063, help="Camera height (m)")
    rs.add_argument(
        "--rs-map-interval",
        type=float,
        default=rs_cfg.MAP_UPDATE_INTERVAL_SEC,
        help="Seconds between occupancy updates",
    )

    return parser


def _build_tof_args(args: argparse.Namespace) -> List[str]:
    tof_args = [
        "--port",
        args.tof_port,
        "--baudrate",
        str(args.tof_baudrate),
        "--fps",
        str(args.tof_fps),
        "--min-distance",
        f"{args.tof_min_distance}",
        "--max-distance",
        f"{args.tof_max_distance}",
        "--min-area",
        str(args.tof_min_area),
    ]
    if args.tof_verbose:
        tof_args.append("--verbose")
    return tof_args


def _build_realsense_args(args: argparse.Namespace) -> List[str]:
    rs_args = [
        "--tcp-ip",
        args.rs_tcp_ip,
        "--tcp-port",
        str(args.rs_tcp_port),
        "--cam-pitch",
        f"{args.rs_cam_pitch}",
        "--cam-height",
        f"{args.rs_cam_height}",
        "--map-interval",
        f"{args.rs_map_interval}",
    ]
    if args.rs_no_tcp:
        rs_args.append("--no-tcp")
    return rs_args


def main(argv: Optional[Sequence[str]] = None):
    parser = build_parser()
    args = parser.parse_args(argv)

    tof_args = None if args.disable_tof else _build_tof_args(args)
    rs_args = None if args.disable_realsense else _build_realsense_args(args)

    system = make_default_system(tof_args=tof_args, realsense_args=rs_args)
    system.run(
        enable_tof=not args.disable_tof,
        enable_realsense=not args.disable_realsense,
        join=True,
    )


if __name__ == "__main__":
    main()
