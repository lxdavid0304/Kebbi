from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

import numpy as np


DistanceConverter = Callable[[int], float]


@dataclass
class PacketProcessingResult:
    valid: bool
    consumed: int
    raw_image: Optional[np.ndarray] = None
    distance_image: Optional[np.ndarray] = None


class PacketParser:
    """Handles MaixSense packet structure parsing independent of controller logic."""

    def __init__(self, distance_converter: DistanceConverter):
        self.distance_converter = distance_converter

    def find_sync_bytes(self, data: bytes, start_pos: int = 0) -> int:
        for i in range(start_pos, len(data) - 1):
            if data[i] == 0x00 and data[i + 1] == 0xFF:
                return i
        return -1

    def parse_packet_header(
        self, data: bytes, start_pos: int
    ) -> Optional[Tuple[int, int]]:
        if len(data) < start_pos + 20:
            return None
        pos = start_pos + 2
        packet_length = struct.unpack("<H", data[pos : pos + 2])[0]
        pos += 2 + 16
        return packet_length, pos

    def verify_checksum(
        self, data: bytes, start_pos: int, end_pos: int, expected_checksum: int
    ) -> bool:
        calculated_sum = sum(data[start_pos:end_pos]) & 0xFF
        return calculated_sum == expected_checksum

    def parse_image_data(
        self, data: bytes, start_pos: int, image_size: int
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if len(data) < start_pos + image_size:
            return None

        image_bytes = data[start_pos : start_pos + image_size]
        if len(image_bytes) >= 20000:
            width = height = 100
            image_bytes = image_bytes[:20000]
        elif len(image_bytes) >= 10000:
            width = height = 100
            image_bytes = image_bytes[:10000]
        else:
            width = height = 100
            if len(image_bytes) < 10000:
                padded_bytes = bytearray(image_bytes)
                padded_bytes.extend([0] * (10000 - len(image_bytes)))
                image_bytes = bytes(padded_bytes)
            else:
                image_bytes = image_bytes[:10000]

        raw_image = np.frombuffer(image_bytes[: width * height], dtype=np.uint8).reshape((height, width))
        distance_image = np.zeros_like(raw_image, dtype=np.float32)
        for y in range(height):
            for x in range(width):
                distance_image[y, x] = self.distance_converter(int(raw_image[y, x]))
        return raw_image, distance_image

    def process_packet(self, data: bytes, start_pos: int) -> Optional[PacketProcessingResult]:
        header_result = self.parse_packet_header(data, start_pos)
        if not header_result:
            if len(data) < start_pos + 20:
                return None
            return PacketProcessingResult(valid=False, consumed=2)

        packet_length, image_start_pos = header_result
        total_packet_size = 2 + 2 + 16 + packet_length - 18 + 1 + 1

        if len(data) < start_pos + total_packet_size:
            return None

        image_size = packet_length - 18
        if image_size < 625 or image_size > 10000:
            return PacketProcessingResult(valid=False, consumed=total_packet_size)

        checksum_index = image_start_pos + image_size
        checksum_start = start_pos + 2
        expected_checksum = data[checksum_index]
        if not self.verify_checksum(data, checksum_start, checksum_index, expected_checksum):
            return PacketProcessingResult(valid=False, consumed=total_packet_size)

        image_result = self.parse_image_data(data, image_start_pos, image_size)
        if not image_result:
            return PacketProcessingResult(valid=False, consumed=total_packet_size)

        raw_image, distance_image = image_result
        return PacketProcessingResult(
            valid=True,
            consumed=total_packet_size,
            raw_image=raw_image,
            distance_image=distance_image,
        )
