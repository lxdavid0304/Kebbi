# -*- coding: utf-8 -*-
"""
MaixSense-A010 串口通信程式
用於與 MaixSense 設備通信，發送 AT+DISP=3 命令並解析返回的圖像數據
"""

import serial
import time
import struct
import sys
import threading
import cv2
import numpy as np
from typing import Optional, Tuple, List
import traceback
import socket
#import sounddevice as sd

class AudioFeedback:
    def __init__(self):
        self.sample_rate = 44100
        self.last_beep_time = 0
        self.last_duration = 0  # 記錄上一次聲音的長度

    def beep(self, frequency=1000, duration=0.1, channel='both', gap=0.1):
        """
        發出嗶聲 (帶有冷卻機制)
        Args:
            frequency: 頻率
            duration: 聲音持續時間
            channel: 'left', 'right', 'both'
            gap: 聲音之間的間隔時間 (秒) -> 控制嗶...嗶...的節奏
        """
        return  # 暫時禁用聲音功能，避免依賴問題
        current_time = time.time()
        
        # 關鍵邏輯：如果 (現在時間 - 上次播放時間) 還不到 (聲音長度 + 間隔時間)
        # 就直接忽略這次請求，不做任何事
        if current_time - self.last_beep_time < (duration + gap):
            return

        # --- 產生聲音波形 ---
        t = np.linspace(0, duration, int(self.sample_rate * duration), False)
        tone = np.sin(frequency * t * 2 * np.pi)
        
        # 淡入淡出 (避免爆音)
        fade_length = int(self.sample_rate * 0.01)
        if len(tone) > fade_length * 2:
            fade_in = np.linspace(0, 1, fade_length)
            fade_out = np.linspace(1, 0, fade_length)
            tone[:fade_length] *= fade_in
            tone[-fade_length:] *= fade_out

        # 聲道處理
        stereo_sound = np.zeros((len(tone), 2), dtype=np.float32)
        if channel == 'left':
            stereo_sound[:, 0] = tone 
        elif channel == 'right':
            stereo_sound[:, 1] = tone
        else:
            stereo_sound[:, 0] = tone
            stereo_sound[:, 1] = tone

        # 播放並更新時間
        sd.play(stereo_sound, self.sample_rate)
        self.last_beep_time = current_time

class MaixSenseController:
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 57600,
                    tcp_host: str = "172.20.10.8", tcp_port: int = 8888):
        """
        初始化 MaixSense 控制器
        
        Args:
            port: 串口設備路徑
            baudrate: 波特率
            tcp_host: 凱比機器人的 IP 位址
            tcp_port: 凱比機器人的 TCP 端口
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn: Optional[serial.Serial] = None
        self.running = False
        self.current_unit = 0  # 當前量化單位
        self.audio = AudioFeedback()
        self.vertical_offset_mm = 530.0  # 盲人胸口到鏡頭的垂直高度 (53cm)
        
        # TCP 連線設定
        self.tcp_host = tcp_host
        self.tcp_port = tcp_port
        self.tcp_socket: Optional[socket.socket] = None
        self.tcp_connected = False
        self.last_command_time = 0  # 記錄上次發送指令的時間
        self.command_cooldown = 2.0  # 指令冷卻時間（秒），避免重複發送
        
    def connect(self) -> bool:
        """
        連接到串口設備和 TCP 伺服器
        
        Returns:
            bool: 連接是否成功
        """
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            self.send_at_command("AT+BAUD=57600")
            print(f"成功連接到 {self.port}，波特率：{self.baudrate}")
            
            # 嘗試連接到 TCP 伺服器
            self.connect_tcp()
            
            return True
        except serial.SerialException as e:
            print(f"連接串口失敗：{e}")
            return False
    
    def connect_tcp(self) -> bool:
        """
        連接到凱比機器人的 TCP 伺服器
        
        Returns:
            bool: 連接是否成功
        """
        try:
            self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # 設定 TCP 參數以保持連線
            self.tcp_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            self.tcp_socket.settimeout(5.0)
            self.tcp_socket.connect((self.tcp_host, self.tcp_port))
            self.tcp_socket.settimeout(None)
            self.tcp_connected = True
            print(f"✅ 成功連接到凱比機器人 TCP: {self.tcp_host}:{self.tcp_port}")
            return True
        except Exception as e:
            print(f"⚠️ 連接凱比機器人 TCP 失敗：{e}")
            self.tcp_connected = False
            return False
    
    def send_tts_command(self, text: str) -> bool:
        """
        發送 TTS 語音指令給凱比機器人
        
        Args:
            text: 要唸出的文字
            
        Returns:
            bool: 發送是否成功
        """

        
        # 檢查冷卻時間，避免頻繁發送
        current_time = time.time()
        if current_time - self.last_command_time < self.command_cooldown:
            return False
        
        if not self.tcp_connected or not self.tcp_socket:
            print("⚠️ TCP 未連線，嘗試重新連接...")
            if not self.connect_tcp():
                return False
        
        try:
            # 發送 TTS 指令，格式：tts:文字內容\n
            command = f"tts:{text}\n"
            self.tcp_socket.sendall(command.encode('utf-8'))
            # 確保數據立即發送（雖然已設定 TCP_NODELAY）
            try:
                self.tcp_socket.send(b'')  # Flush
            except:
                pass
            print(f"📢 發送語音指令: {text}")
            self.last_command_time = current_time
            return True
        except BrokenPipeError:
            print(f"⚠️ TCP 連線已斷開，嘗試重新連接...")
            self.tcp_connected = False
            try:
                self.tcp_socket.close()
            except:
                pass
            self.tcp_socket = None
            # 立即重試一次
            if self.connect_tcp():
                try:
                    command = f"tts:{text}\n"
                    self.tcp_socket.sendall(command.encode('utf-8'))
                    print(f"📢 重新發送語音指令: {text}")
                    self.last_command_time = current_time
                    return True
                except:
                    pass
            return False
        except Exception as e:
            print(f"❌ 發送 TCP 指令失敗：{e}")
            self.tcp_connected = False
            try:
                self.tcp_socket.close()
            except:
                pass
            self.tcp_socket = None
            return False
    
    def disconnect(self):
        """斷開串口和 TCP 連接"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("串口連接已斷開")
        
        if self.tcp_socket:
            try:
                self.tcp_socket.close()
                print("TCP 連接已斷開")
            except:
                pass
            self.tcp_socket = None
            self.tcp_connected = False
    
    def send_at_command(self, command: str) -> bool:
        """
        發送 AT 命令
        
        Args1:
            command: AT 命令字符串
            
        Returns:
            bool: 發送是否成功
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            print("串口未連接")
            return False
        
        try:
            # 發送命令，添加回車換行
            cmd_bytes = (command + '\r\n').encode('utf-8')
            self.serial_conn.write(cmd_bytes)
            print(f"發送命令：{command}")
            
            # 等待響應
            time.sleep(0.1)
            if self.serial_conn.in_waiting > 0:
                response = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                print(f"設備響應：{response.strip()}")
            
            return True
        except Exception as e:
            print(f"發送命令失敗：{e}")
            return False
    
    def enable_display_output(self) -> bool:
        """
        發送 AT+DISP1=3 命令，開啟 LCD 和 USB 輸出
        
        Returns:
            bool: 命令發送是否成功
        """
        return self.send_at_command("AT+DISP=3")
    def set_fps(self, fps: int) -> bool:
        """
        發送 AT+FPS=1 命令, change the framerate
        
        Returns:
            bool: 命令發送是否成功
        """
        return self.send_at_command("AT+FPS=%d"%fps)
    
    def query_unit(self) -> bool:
        """
        查詢當前 UNIT 值
        
        Returns:
            bool: 查詢是否成功
        """
        return self.send_at_command("AT+UNIT?")
    
    def find_sync_bytes(self, data: bytes, start_pos: int = 0) -> int:
        """
        尋找同步字節 0x00 0xFF
        
        Args:
            data: 數據字節
            start_pos2: 開始搜索位置
            
        Returns:
            int: 同步字節位置，-1 表示未找到
        """
        for i in range(start_pos, len(data) - 1):
            if data[i] == 0x00 and data[i + 1] == 0xFF:
                return i
        return -1
    
    def parse_packet_header(self, data: bytes, start_pos: int) -> Optional[Tuple[int, int, dict]]:
        """
        解析數據包頭部
        
        Args:
            data: 數據字節
            start_pos: 包頭開始位置
            
        Returns:
            Tuple[包長度, 下一個數據位置, 頭部信息] 或 None
        """
        if len(data) < start_pos + 20:  # 包頭至少需要 20 字節
            return None
        
        try:
            # 跳過同步字節 (0x00 0xFF)
            pos = start_pos + 2
            
            # 讀取包長度 (2字節)
            packet_length = struct.unpack('<H', data[pos:pos+2])[0]
            pos += 2
            
            # 讀取其他頭部信息 (16字節)
            header_info = {
                'packet_length': packet_length,
                'header_data': data[pos:pos+16].hex()
            }
            pos += 16
            
            return packet_length, pos, header_info
            
        except struct.error as e:
            print(f"解析包頭失敗：{e}")
            return None
    
    def calculate_distance(self, pixel_value: int) -> float:
        """
        根據像素值計算距離
        
        Args:
            pixel_value: 像素值
            
        Returns8:
            float: 距離值 (mm)
        """
        if self.current_unit != 0:
            # UNIT 非 0：距離 = p × UNIT
            return pixel_value * self.current_unit
        else:
            # UNIT 為 0：距離 = (p/5.1)²
            return (pixel_value / 5.1) ** 2
    
    def parse_image_data(self, data: bytes, start_pos: int, image_size: int) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        解析圖像數據並轉換為原始數據和距離值
        
        Args:
            data: 數據字節
            start_pos: 圖像數據開始位置
            image_size: 圖像數據大小
            
        Returns:
            Tuple[原始圖像數組, 距離圖像數組] 或 None
        """
        if len(data) < start_pos + image_size:
            return None
        
        try:
            # 提取圖像數據
            image_bytes = data[start_pos:start_pos + image_size]
            
            # MaixSense-A010 的解析度固定為 100x100，每個像素 2 字節
            # 但實際接收到的數據可能不完整，所以需要處理不同情況
            if len(image_bytes) >= 20000:  # 100x100 * 2 bytes = 20000
                width = height = 100
                # 使用前 20000 字節作為圖像數據
                image_bytes = image_bytes[:20000]
            elif len(image_bytes) >= 10000:  # 可能是 100x100 * 1 byte
                width = height = 100
                # 使用前 10000 字節
                image_bytes = image_bytes[:10000]
            else:
                # 對於其他大小，嘗試推算最接近 100x100 的處理方式
                width = height = 100
                print(f"數據大小不標準：{len(image_bytes)} 字節，期望 10000 或 20000 字節")
                # 如果數據不足，用零填充
                if len(image_bytes) < 10000:
                    padded_bytes = bytearray(image_bytes)
                    padded_bytes.extend([0] * (10000 - len(image_bytes)))
                    image_bytes = bytes(padded_bytes)
                else:
                    image_bytes = image_bytes[:10000]
            
            print(f"解析度：{width}x{height}，處理數據大小：{len(image_bytes)}")
            
            # 轉換為 numpy 數組
            raw_image = np.frombuffer(image_bytes[:width*height], dtype=np.uint8)
            raw_image = raw_image.reshape((height, width))
            
            # 計算距離圖像
            distance_image = np.zeros_like(raw_image, dtype=np.float32)
            for y in range(height):
                for x in range(width):
                    pixel_value = raw_image[y, x]
                    distance_image[y, x] = self.calculate_distance(pixel_value)
            
            return raw_image, distance_image
            
        except Exception as e:
            print(f"解析圖像數據失敗：{e}")
            return None
    
    def verify_checksum(self, data: bytes, start_pos: int, end_pos: int, expected_checksum: int) -> bool:
        """
        驗證校驗和
        
        Args:
            data: 數據字節
            start_pos0: 開始位置
            end_pos: 結束位置
            expected_checksum: 期望的校驗和
            
        Returns:
            bool: 校驗是否通過
        """
        calculated_sum = sum(data[start_pos:end_pos]) & 0xFF
        return calculated_sum == expected_checksum
    
    def process_data_packet(self, data: bytes, start_pos: int) -> Optional[int]:
        """
        處理完整的數據包
        
        Args:
            data: 數據字節
            start_pos: 包開始位置
            
        Returns:
            int: 下一個包的搜索位置 或 None
        """
        # 解析包頭
        header_result = self.parse_packet_header(data, start_pos)
        if not header_result:
            return None
        
        packet_length, image_start_pos, header_info = header_result
        
        # 計算包的總長度 (包頭 + 圖像數據 + 校驗 + 包尾)
        total_packet_size = 2 + 2 + 16 + packet_length - 18 + 1 + 1  # 同步字節 + 長度 + 頭部 + 圖像 + 校驗 + 包尾
        
        if len(data) < start_pos + total_packet_size:
            return None  # 數據不完整
        
        # 計算圖像數據大小
        image_size = packet_length - 18  # 總長度減去頭部信息長度
        
        # 只處理合理大小的圖像數據
        if image_size < 625 or image_size > 10000:  # 25x25 到 100x100
            return start_pos + total_packet_size
        
        # 解析圖像數據
        image_result = self.parse_image_data(data, image_start_pos, image_size)
        
        if image_result:
            raw_image, distance_image = image_result
            print(f"✅ 成功解析圖像數據，分辨率：{raw_image.shape[1]}x{raw_image.shape[0]}")
            
            # 使用 OpenCV 顯示圖像
            self.display_images(raw_image, distance_image)
            
            # 顯示距離統計
            min_dist = np.min(distance_image)
            max_dist = np.max(distance_image)
            mean_dist = np.mean(distance_image)
            print(f"距離統計 - 最小值: {min_dist:.1f}mm, 最大值: {max_dist:.1f}mm, 平均值: {mean_dist:.1f}mm")
        
        return start_pos + total_packet_size
    
    def find_person(self, distance_image: np.ndarray) -> Optional[Tuple[int, int, int, int, int, int, float]]:
        """
        分析距離圖像，找出最顯著的物體（假設為人）。
        
        Args:
            distance_image3: 原始距離數據 (float, 100x100)
            
        Returns:
            如果找到物體，返回 (x, y, w, h, cx, cy, median_distance)
            否則返回 None
        """
        try:
            # --- 1. 調整感興趣的距離範圍 ---
            # 根據實際測試調整，例如：30公分到2.5公尺。
            # 靠近時可以調低MAX_DIST_MM, 遠離時可以調高
            MIN_DIST_MM = 500.0  # 提高最小距離，排除太近的雜訊 (例如手靠近鏡頭)
            MAX_DIST_MM = 1600.0 # 最大偵測距離 1.2公尺

            # 2. 創建二進制遮罩 (有效距離內的像素)
            mask = cv2.inRange(distance_image, MIN_DIST_MM, MAX_DIST_MM)
            
            # --- 3. 清理遮罩 (形態學操作，增加迭代次數，增強效果) ---
            kernel = np.ones((5, 5), np.uint8)
            
            # 增加 OPEN 的迭代次數以更有效地去除小噪點
            mask_cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=3) 
            # 增加 CLOSE 的迭代次數以更好地填充物體內部的空洞，連接分離的部分
            mask_cleaned = cv2.morphologyEx(mask_cleaned, cv2.MORPH_CLOSE, kernel, iterations=4) 

            # 4. 尋找輪廓
            contours, _ = cv2.findContours(mask_cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                return None  # 沒有找到任何物體

            # --- 5. 過濾和選擇輪廓 ---
            
            # 儲存符合條件的潛在人物輪廓
            potential_person_contours = []

            for contour in contours:
                area = cv2.contourArea(contour)
                # 過濾掉太小的輪廓。100x100 圖像中，人可能佔幾百甚至上千像素。
                # 這裡將閾值從 25 提高到 500，請根據實際畫面大小和人距離調整。
                if area < 250: # 假設人的最小面積，可以嘗試 100, 200, 500 等值
                    continue
                
                # 計算邊界框
                x, y, w, h = cv2.boundingRect(contour)
                
                # 過濾：長寬比 (Aspect Ratio)
                # 人形狀通常是高而窄的，例如長寬比在 0.4 到 1.5 之間 (高/寬)
                if w == 0 or h == 0: # 避免除以零
                    continue
                aspect_ratio = float(h) / w 
                if not (0.8 < aspect_ratio < 2.5): # 調整這個範圍以符合您眼中「人」的比例
                    continue

                # 過濾：實心度 (Solidity)
                # 實心度 = 輪廓面積 / 凸包面積
                # 用於排除不規則形狀或內部有大洞的輪廓
                hull = cv2.convexHull(contour)
                hull_area = cv2.contourArea(hull)
                if hull_area == 0:
                    continue
                solidity = float(area) / hull_area
                if solidity < 0.7: # 調整這個閾值，0.7表示至少70%是實心的
                    continue
                
                potential_person_contours.append(contour)
            
            if not potential_person_contours:
                return None # 沒有符合所有條件的輪廓

            # 在所有符合條件的輪廓中，選擇最大的作為人
            largest_contour = max(potential_person_contours, key=cv2.contourArea)
            
            # 6. 獲取位置和距離
            x, y, w, h = cv2.boundingRect(largest_contour)
            cx = x + w // 2
            cy = y + h // 2
            
            person_roi = distance_image[y:y+h, x:x+w]
            # 注意：mask_cleaned 必須是與 distance_image 相同大小的二值圖
            # 確保這裡的索引範圍正確，防止越界
            valid_distances = person_roi[mask_cleaned[y:y+h, x:x+w] > 0]
            
            if valid_distances.size == 0:
                # 再次檢查，如果過濾後沒有有效距離，也返回 None
                # 這發生在輪廓區域恰好都是無效深度數據時
                return None

            person_distance = np.median(valid_distances)
            
            return (x, y, w, h, cx, cy, person_distance)

        except Exception as e:
            print(f"尋找人物時出錯: {e}")
            return None
    
    def display_images(self, raw_image: np.ndarray, distance_image: np.ndarray):
        """
        使用 OpenCV 顯示原始圖像和距離圖像，並加入左右位置與距離導引提示
        """
        try:
            # 不進行翻轉，保持原始方向與 LCD 一致
            raw_flipped = raw_image.copy()
            distance_flipped = distance_image.copy()
            
            # 正規化原始圖像到 0-255 範圍
            raw_display = cv2.normalize(raw_flipped, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            # 對距離圖像進行特殊處理以匹配 LCD 顯示
            valid_mask = (distance_flipped > 0) & (distance_flipped < 2500)
            
            processed_distance = distance_flipped.copy()
            processed_distance[~valid_mask] = 0
            
            if np.max(processed_distance[valid_mask]) > 0:
                max_valid_dist = np.max(processed_distance[valid_mask])
                inverted_distance = np.zeros_like(processed_distance)
                inverted_distance[valid_mask] = max_valid_dist - processed_distance[valid_mask]
                distance_normalized = cv2.normalize(inverted_distance, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            else:
                distance_normalized = np.zeros_like(processed_distance, dtype=np.uint8)
            
            distance_colored = cv2.applyColorMap(distance_normalized, cv2.COLORMAP_HOT)
            distance_colored[~valid_mask] = [0, 0, 0]
            
            # 調整圖像大小以便顯示（放大到 400x400）
            scale_factor = 4
            target_size = 400
            raw_resized = cv2.resize(raw_display, (target_size, target_size), interpolation=cv2.INTER_NEAREST)
            distance_resized = cv2.resize(distance_colored, (target_size, target_size), interpolation=cv2.INTER_NEAREST)
            
            # --- 人物偵測與位置導引 ---
            detection_result = self.find_person(distance_image)
            
            # 定義閾值
            # 視野60度 (-30° ~ +30°)，100像素寬
            # 每像素 = 0.6度
            LEFT_WARNING = 17      # -20度警告線 (像素17 = -20度)
            RIGHT_WARNING = 83     # +20度警告線 (像素83 = +20度)
            LEFT_LIMIT = 0         # -30度極限 (超出偵測範圍)
            RIGHT_LIMIT = 100      # +30度極限 (超出偵測範圍)
            MAX_DIST_THRESHOLD = 1200.0 # 最大距離 1.2 公尺 (1200mm)
            
            if detection_result:
                import math
                x, y, w, h, cx, cy, person_distance = detection_result
                
                # === 使用畢氏定理計算水平距離 ===
                if person_distance > self.vertical_offset_mm:
                    # 畢氏定理：horizontal = √(diagonal² - vertical²)
                    horizontal_distance = math.sqrt(
                        person_distance**2 - self.vertical_offset_mm**2
                    )
                else:
                    # 斜線距離小於垂直高度 -> 盲人太靠近或在鏡頭下方
                    horizontal_distance = 0.0
                
                # === 判斷狀態（優先級：太近 > 太遠 > 超出範圍 > 太左/太右 > 正常）===
                if person_distance <= self.vertical_offset_mm:
                    distance_status = "TOO_CLOSE"
                elif person_distance > MAX_DIST_THRESHOLD:
                    distance_status = "TOO_FAR"
                elif cx <= LEFT_LIMIT:
                    distance_status = "OUT_OF_RANGE_LEFT"  # 超出左側30度
                elif cx >= RIGHT_LIMIT:
                    distance_status = "OUT_OF_RANGE_RIGHT"  # 超出右側30度
                elif cx < LEFT_WARNING:
                    distance_status = "TOO_LEFT"  # 在-30度~-20度之間
                elif cx > RIGHT_WARNING:
                    distance_status = "TOO_RIGHT"  # 在+20度~+30度之間
                else:
                    distance_status = "NORMAL"  # 在-20度~+20度之間
                
                # 在終端印出距離資訊
                print("=" * 60)
                print(f"📏 距離測量：")
                print(f"   斜線距離 (鏡頭直接測量): {person_distance:.1f} mm ({person_distance/10:.1f} cm)")
                print(f"   垂直高度差 (胸口到鏡頭): {self.vertical_offset_mm:.1f} mm ({self.vertical_offset_mm/10:.1f} cm)")
                print(f"   水平距離 (機器人到人):   {horizontal_distance:.1f} mm ({horizontal_distance/10:.1f} cm)")
                print(f"   狀態: {distance_status}")
                print("=" * 60)
                
                # 座標縮放
                x_scaled, y_scaled = x * scale_factor, y * scale_factor
                w_scaled, h_scaled = w * scale_factor, h * scale_factor
                cx_scaled, cy_scaled = cx * scale_factor, cy * scale_factor
                
                # 1. 繪製邊界框 (綠色)
                cv2.rectangle(distance_resized, (x_scaled, y_scaled), (x_scaled + w_scaled, y_scaled + h_scaled), (0, 255, 0), 2)
                # 2. 繪製中心點 (紅色)
                cv2.circle(distance_resized, (cx_scaled, cy_scaled), 5, (0, 0, 255), -1)
                
                # 3. 判斷位置並生成導引訊息 (優先級：左右 -> 距離)
                guide_text = ""
                guide_color = (0, 255, 0)
            
                # 定義聲音參數
                BEEP_FREQ = 2500  # 嗶聲頻率
                SHORT_BEEP = 0.1 # 短嗶聲長度
                SHORT_GAP = 0.2
                LONG_BEEP = 0.5  # 長嗶聲長度
                LONG_GAP = 0.3
                is_too_left = 0
                is_too_right = 0


                # 根據狀態設定警告訊息（修正鏡像邏輯）
                if distance_status == "TOO_CLOSE":
                    guide_text = "⚠️ TOO CLOSE! DANGER ⚠️"
                    guide_color = (0, 0, 255)
                
                elif distance_status == "TOO_FAR":
                    guide_text = "⚠️ OUT OF RANGE! Move Forward"
                    guide_color = (0, 0, 255)
                    self.audio.beep(frequency=2500, duration=LONG_BEEP, channel='both', gap=LONG_GAP)
                
                elif distance_status == "OUT_OF_RANGE_LEFT":
                    guide_text = "⚠️ OUT OF RANGE! Move Left <<<"  # 修正：畫面左側要往左移
                    guide_color = (0, 0, 255)
                    is_too_left = 1
                    self.send_tts_command("向左移動")
                
                elif distance_status == "OUT_OF_RANGE_RIGHT":
                    guide_text = "⚠️ OUT OF RANGE! Move Right >>>"  # 修正：畫面右側要往右移
                    guide_color = (0, 0, 255)
                    is_too_right = 1
                    self.send_tts_command("向右移動")
                
                elif distance_status == "TOO_LEFT":
                    guide_text = "Move Left <<<"  # 修正：畫面左側要往左移
                    guide_color = (255, 165, 0)  # 橘色提醒
                    is_too_left = 1
                    self.audio.beep(frequency=BEEP_FREQ, duration=SHORT_BEEP, channel='left', gap=SHORT_GAP)
                    self.send_tts_command("向左移動")
                
                elif distance_status == "TOO_RIGHT":
                    guide_text = "Move Right >>>"  # 修正：畫面右側要往右移
                    guide_color = (255, 165, 0)  # 橘色提醒
                    is_too_right = 1
                    self.audio.beep(frequency=BEEP_FREQ, duration=SHORT_BEEP, channel='right', gap=SHORT_GAP)
                    self.send_tts_command("向右移動")
                
                else:  # NORMAL
                    guide_text = "✓ Position Perfect!"
                    guide_color = (0, 255, 0)
                
            
                # 寫入檔案 (使用水平距離)
                f=open('/tmp/human_position.txt','w')
                f.write(f"{horizontal_distance:.1f},{is_too_left},{is_too_right},{distance_status}\n")
                f.write(f"DIAGONAL:{person_distance:.1f}\n")
                f.write(f"VERTICAL_OFFSET:{self.vertical_offset_mm:.1f}\n")
                f.write("XXX\n")
                f.close()
            

                # 4. 計算當前角度（修正：右邊為正，左邊為負）
                # cx範圍: 0-100，左邊0對應-30度，右邊100對應+30度
                # 但視覺上：畫面右側應該是正角度，左側應該是負角度
                current_angle = (50 - cx) * 0.6  # 反轉：右邊為正，左邊為負
                
                # 5. 根據狀態決定顯示顏色
                # 紅色：距離或角度任一超出範圍（太近、太遠、超出±30度）
                # 橘色：距離正常，但角度在±20-30度之間
                # 綠色：距離和角度都正常
                if distance_status in ["TOO_CLOSE", "TOO_FAR", "OUT_OF_RANGE_LEFT", "OUT_OF_RANGE_RIGHT"]:
                    display_color = (0, 0, 255)  # 紅色：嚴重警告
                elif distance_status in ["TOO_LEFT", "TOO_RIGHT"]:
                    display_color = (0, 165, 255)  # 橘色(BGR)：需要調整
                else:  # NORMAL
                    display_color = (0, 255, 0)  # 綠色：完美
                
                # 6. 準備顯示文字
                dist_text = f"Dist: {person_distance:.0f}mm"
                angle_text = f"Angle: {current_angle:+.1f}°"
                
                # 7. 狀態文字
                if distance_status == "TOO_CLOSE":
                    status_text = "TOO CLOSE!"
                elif distance_status == "TOO_FAR":
                    status_text = "TOO FAR!"
                elif distance_status == "OUT_OF_RANGE_LEFT":
                    status_text = "OUT OF RANGE (LEFT)"
                elif distance_status == "OUT_OF_RANGE_RIGHT":
                    status_text = "OUT OF RANGE (RIGHT)"
                elif distance_status == "TOO_LEFT":
                    status_text = "Move Left"
                elif distance_status == "TOO_RIGHT":
                    status_text = "Move Right"
                else:  # NORMAL
                    status_text = "PERFECT"
                
                # 8. 繪製資訊到視窗固定位置
                # 距離：左上角
                cv2.putText(distance_resized, dist_text, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, display_color, 2)
                
                # 角度：右上角
                angle_text_size = cv2.getTextSize(angle_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
                cv2.putText(distance_resized, angle_text, (target_size - angle_text_size[0] - 10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, display_color, 2)
                
                # 狀態：正下方中央
                status_text_size = cv2.getTextSize(status_text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)[0]
                status_x = (target_size - status_text_size[0]) // 2
                cv2.putText(distance_resized, status_text, (status_x, target_size - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, display_color, 3)
                            
                # 原本的底部藍色導引文字已刪除，改用上方的紅/橘/綠色狀態顯示
                            
            else:
                # 沒偵測到人
                raw_resized_bgr = cv2.cvtColor(raw_resized, cv2.COLOR_GRAY2BGR)
                cv2.putText(distance_resized, "No Person Detected", (10, 390), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

            # 轉為 BGR 以便 hstack
            raw_resized_bgr = cv2.cvtColor(raw_resized, cv2.COLOR_GRAY2BGR) if len(raw_resized.shape) == 2 else raw_resized
            
            # 添加標題區塊
            raw_with_title = np.zeros((430, 400, 3), dtype=np.uint8)
            raw_with_title[30:430, :] = raw_resized_bgr
            cv2.putText(raw_with_title, 'Raw Image', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            distance_with_title = np.zeros((430, 400, 3), dtype=np.uint8)
            distance_with_title[30:430, :] = distance_resized
            cv2.putText(distance_with_title, 'Guide View', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # 並排顯示
            combined = np.hstack([raw_with_title, distance_with_title])
            cv2.imshow('MaixSense ToF - Position Guide', combined)
            
            cv2.waitKey(1)
            
        except Exception as e:
            print(f"顯示圖像時發生錯誤：{e}")
            traceback.print_exc()
    
    def listen_for_data(self):
        """
        監聽串口數據並解析 (包含防錯位機制修正版)
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            print("串口未連接")
            return
        
        print("開始監聽數據...")
        self.running = True
        buffer = bytearray()
        total_bytes_received = 0
        packet_count = 0
        successful_packets = 0
        
        # 定義合理的圖像數據大小範圍 (100x100 解析度)
        # 寬鬆一點的範圍，避免邊界誤差
        MIN_EXPECTED_SIZE = 9000   # 針對 100x100 8bit
        MAX_EXPECTED_SIZE = 21000  # 針對 100x100 16bit
        
        while self.running:
            try:
                # 讀取可用數據
                if self.serial_conn.in_waiting > 0:
                    new_data = self.serial_conn.read(self.serial_conn.in_waiting)
                    buffer.extend(new_data)
                    total_bytes_received += len(new_data)
                else:
                    time.sleep(0.005) # 稍微休眠避免 CPU 滿載
                    continue

                # 循環處理緩衝區
                while len(buffer) > 20: # 至少要有包頭長度
                    
                    # 1. 尋找同步字節
                    sync_pos = self.find_sync_bytes(buffer, 0)
                    
                    if sync_pos == -1:
                        # 沒找到同步頭，但為了防止緩衝區無限膨脹，只保留最後一點數據
                        if len(buffer) > 2048:
                            buffer = buffer[-1024:] 
                        break # 等待更多數據
                    
                    # 2. 移除同步頭之前的垃圾數據
                    if sync_pos > 0:
                        buffer = buffer[sync_pos:]
                        # 重新檢查長度
                        if len(buffer) < 20:
                            break

                    # 3. 【關鍵修正】檢查包長度是否合理
                    try:
                        # 解析包長度 (Bytes 2-3)
                        packet_length = struct.unpack('<H', buffer[2:4])[0]
                        
                        # 計算圖像 payload 大小 (總長 - 表頭18bytes)
                        payload_size = packet_length - 18
                        
                        # --- 嚴格過濾：如果長度不合理，這就是一個「假」的同步頭 ---
                        if not (MIN_EXPECTED_SIZE <= payload_size <= MAX_EXPECTED_SIZE):
                            # print(f"跳過偽造包頭 (Size: {payload_size})")
                            buffer = buffer[1:] # 往後移動 1 byte，重新尋找下一個 0x00 FF
                            continue
                            
                        # 計算完整包的總大小 (同步2 + 長度2 + 資訊16 + 圖像 + 校驗1 + 尾1)
                        # 公式簡化： packet_length + 2(同步) + 2(尾巴校驗)
                        total_packet_size = packet_length + 4 
                        
                        # 檢查是否已接收完整的包
                        if len(buffer) < total_packet_size:
                            break # 數據還不夠，跳出等待更多數據
                            
                        # 4. 處理數據包
                        # 因為我們已經確認過長度合理，這裡可以直接處理
                        result = self.process_data_packet(buffer, 0)
                        
                        if result:
                            successful_packets += 1
                            packet_count += 1
                            # 成功處理，移除這個包的數據
                            buffer = buffer[total_packet_size:]
                        else:
                            # 校驗失敗或其他錯誤
                            print("包校驗失敗，丟棄")
                            buffer = buffer[1:] # 移動 1 byte 重試
                            
                    except (struct.error, IndexError):
                        buffer = buffer[1:]
                        continue
                        
            except KeyboardInterrupt:
                print("\n停止監聽")
                break
            except Exception as e:
                print(f"錯誤：{e}")
                # 出錯時清空緩衝區，避免死循環
                buffer = bytearray()
                time.sleep(0.1)
        
        self.running = False
    
    def start_monitoring(self):
        """
        開始監控數據的線程
        """
        monitor_thread = threading.Thread(target=self.listen_for_data)
        monitor_thread.daemon = True
        monitor_thread.start()
        return monitor_thread

def main():
    """主函數"""
    print("MaixSense-A010 串口通信程式")
    print("=" * 50)
    
    # 創建控制器實例
    controller = MaixSenseController()
    
    try:
        # 連接串口
        while True:
            if not controller.connect():
                print("無法連接到設備，程式退出")
                return
            
            # 查詢當前 UNIT 值
            print("\n查詢當前 UNIT 值...")
            controller.query_unit()
            #time.sleep(1)
            
            # 發送 AT+DISP=3 命令
            print("\n發送 AT+DISP=3 命令...")
            if not controller.enable_display_output():
                print("發送命令失敗")
                return
            if not controller.set_fps(4):
                print("發送命令失敗")
                return
            
            # 等待設備響應
            #time.sleep(2)
            
            # 開始監聽數據
            print("\n開始監聽數據，按 Ctrl+C 停止...")
            #monitor_thread = controller.start_monitoring()
            controller.listen_for_data()
        
        # 等待用戶中斷
        try:
            while controller.running:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n正在停止程式...")
            controller.running = False
        
        # 等待監聽線程結束
        monitor_thread.join(timeout=2)
        
    except Exception as e:
        print(f"程式執行錯誤：{e}")
    
    finally:
        # 斷開連接
        controller.disconnect()
        # 關閉所有 OpenCV 窗口
        cv2.destroyAllWindows()
        print("程式結束")

if __name__ == "__main__":
    main()
