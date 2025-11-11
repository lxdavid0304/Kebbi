from __future__ import annotations

import re
import threading
import time
import traceback
from typing import Callable, List, Optional

import serial

from .detection import PersonDetection, PersonDetectionParams, PersonDetector
from .packets import PacketParser
from .state import PersonRangeReading, publish_person_range
from .visualizer import ImageVisualizer


class MaixSenseController:
    """High-level controller that manages serial IO, packet parsing, detection, and visualization."""

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 57600,
        verbose: bool = False,
        detection_params: Optional[PersonDetectionParams] = None,
        person_state_callback: Optional[Callable[[Optional[PersonRangeReading]], None]] = None,
    ):
        self.port = port
        self.baudrate = baudrate
        self.verbose = verbose
        self.serial_conn: Optional[serial.Serial] = None
        self.running = False
        self.current_unit = 0.0
        self.idle_read_limit = 300

        self.detector = PersonDetector(detection_params or PersonDetectionParams())
        self.visualizer = ImageVisualizer(self.detector)
        self.packet_parser = PacketParser(self.calculate_distance)
        self.person_state_callback = person_state_callback

    # region logging utilities
    def log_debug(self, message: str):
        if self.verbose:
            print(f"[DEBUG] {message}")

    # endregion

    # region serial helpers
    def connect(self) -> bool:
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0,
            )
            self.send_at_command("AT+BAUD=57600")
            print(f"成功連接到 {self.port}，波特率：{self.baudrate}")
            return True
        except serial.SerialException as exc:
            print(f"連接串口失敗：{exc}")
            return False

    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("串口連接已斷開")

    def send_at_command(self, command: str) -> bool:
        if not self.serial_conn or not self.serial_conn.is_open:
            print("串口未連接")
            return False

        try:
            cmd_bytes = (command + "\r\n").encode("utf-8")
            self.serial_conn.write(cmd_bytes)
            self.log_debug(f"發送命令：{command}")
            time.sleep(0.1)
            response = self._read_serial_response()
            if response:
                print(f"設備響應：{response.strip()}")
            return True
        except Exception as exc:
            print(f"發送命令失敗：{exc}")
            return False

    def _read_serial_response(self, timeout: float = 0.5) -> str:
        if not self.serial_conn or not self.serial_conn.is_open:
            return ""
        deadline = time.time() + timeout
        response = bytearray()
        while time.time() < deadline:
            waiting = getattr(self.serial_conn, "in_waiting", 0)
            chunk = self.serial_conn.read(waiting or 1)
            if chunk:
                response.extend(chunk)
                if response.endswith(b"\r\n"):
                    break
            else:
                time.sleep(0.01)
        return response.decode("utf-8", errors="ignore").strip()

    # endregion

    # region device configuration
    def query_unit(self) -> bool:
        if not self.serial_conn or not self.serial_conn.is_open:
            print("串口未連接")
            return False
        try:
            if hasattr(self.serial_conn, "reset_input_buffer"):
                self.serial_conn.reset_input_buffer()
        except serial.SerialException:
            pass

        try:
            self.serial_conn.write(("AT+UNIT?\r\n").encode("utf-8"))
            self.log_debug("發送命令：AT+UNIT?")
        except Exception as exc:
            print(f"發送命令失敗：{exc}")
            return False

        response = self._read_serial_response(timeout=1.0)
        if response:
            print(f"設備響應：{response}")
        else:
            print("未取得 UNIT 回應，保持現有設定")
            return False

        unit_value = self._extract_unit_from_response(response)
        if unit_value is None:
            print(f"無法解析 UNIT 回應：{response}")
            return False
        self.current_unit = unit_value
        print(f"更新 UNIT：{self.current_unit} mm/LSB")
        return True

    def _extract_unit_from_response(self, response: str) -> Optional[float]:
        match = re.search(r"unit[:=]\s*([0-9]+(?:\.[0-9]+)?)", response, re.IGNORECASE)
        if not match:
            match = re.search(r"([0-9]+(?:\.[0-9]+)?)", response)
        if match:
            try:
                return float(match.group(1))
            except ValueError:
                return None
        return None

    def calculate_distance(self, pixel_value: int) -> float:
        if self.current_unit:
            return pixel_value * self.current_unit
        return (pixel_value / 5.1) ** 2

    def enable_display_output(self) -> bool:
        return self.send_at_command("AT+DISP=3")

    def set_fps(self, fps: int) -> bool:
        return self.send_at_command(f"AT+FPS={fps}")

    # endregion

    # region packet processing
    def listen_for_data(self):
        if not self.serial_conn or not self.serial_conn.is_open:
            print("串口未連接")
            return

        print("開始監聽數據...")
        self.running = True
        buffer = bytearray()
        total_bytes_received = 0
        packet_count = 0
        successful_packets = 0
        idle_reads = 0

        while self.running:
            try:
                bytes_waiting = getattr(self.serial_conn, "in_waiting", 0)
                read_size = bytes_waiting if bytes_waiting > 0 else 512
                new_data = self.serial_conn.read(read_size)
                if not new_data:
                    idle_reads += 1
                    if idle_reads > self.idle_read_limit:
                        self.log_debug("連續多次無數據，保持連線等待新資料")
                        idle_reads = 0
                    continue

                idle_reads = 0
                buffer.extend(new_data)
                total_bytes_received += len(new_data)

                if packet_count and packet_count % 10 == 0:
                    self.log_debug(
                        f"接收統計：總計 {total_bytes_received} 字節，處理了 {packet_count} 個包，成功 {successful_packets} 個"
                    )

                while len(buffer) > 20:
                    sync_pos = self.packet_parser.find_sync_bytes(buffer, 0)
                    if sync_pos == -1:
                        if len(buffer) > 1024:
                            buffer = buffer[-1024:]
                        break
                    if sync_pos > 0:
                        buffer = buffer[sync_pos:]

                    if len(buffer) < 20:
                        break

                    packet_result = self.packet_parser.process_packet(buffer, 0)
                    if packet_result is None:
                        break

                    packet_count += 1
                    if packet_result.valid and packet_result.raw_image is not None:
                        successful_packets += 1
                        detections = self.visualizer.display(
                            packet_result.raw_image, packet_result.distance_image  # type: ignore[arg-type]
                        )
                        self._handle_detections(detections)
                        print(
                            f"✅ 成功解析圖像數據，分辨率："
                            f"{packet_result.raw_image.shape[1]}x{packet_result.raw_image.shape[0]}"
                        )
                    buffer = buffer[packet_result.consumed :]

                time.sleep(0.001)

            except KeyboardInterrupt:
                print("\n收到中斷信號，停止監聽")
                print(
                    f"統計信息：總接收 {total_bytes_received} 字節，"
                    f"處理了 {packet_count} 個數據包，成功 {successful_packets} 個"
                )
                break
            except Exception as exc:
                print(f"監聽數據時發生錯誤：{exc}\n")
                print(traceback.format_exc())
                self.connect()
                continue

        self.running = False

    def start_monitoring(self) -> threading.Thread:
        monitor_thread = threading.Thread(target=self.listen_for_data)
        monitor_thread.daemon = True
        monitor_thread.start()
        return monitor_thread

    def _handle_detections(self, detections: Optional[List[PersonDetection]]) -> None:
        reading: Optional[PersonRangeReading] = None
        if detections:
            best = detections[0]
            reading = PersonRangeReading(
                distance_m=float(best.median_distance) / 1000.0,
                bbox=(best.x, best.y, best.w, best.h),
                area=float(best.area),
                timestamp=time.time(),
            )

        publish_person_range(reading)
        if self.person_state_callback:
            try:
                self.person_state_callback(reading)
            except Exception as exc:
                self.log_debug(f"person_state_callback 執行失敗：{exc}")

    # endregion


def build_default_controller(verbose: bool = False) -> MaixSenseController:
    detection_params = PersonDetectionParams()
    return MaixSenseController(verbose=verbose, detection_params=detection_params)
