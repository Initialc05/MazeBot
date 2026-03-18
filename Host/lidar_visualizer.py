#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RPLidar C1 上位机可视化程序
- 通过HC-04蓝牙模块接收雷达数据
- 实时显示扫描点云
- 简单SLAM栅格地图构建
"""

import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import TextBox, Button
from collections import deque
import time
import json
from datetime import datetime
import os
from PIL import Image
import threading
import queue

# ==================== 配置参数 ====================
# 调试模式选择
USE_BLUETOOTH_MODE = True  # True: 蓝牙模式 (COM12), False: USB串口调试模式 (COM3)

# 串口配置
if USE_BLUETOOTH_MODE:
    SERIAL_PORT = 'COM12'  # 蓝牙串口
    BAUD_RATE = 921600     # 蓝牙波特率
else:
    SERIAL_PORT = 'COM7'   # USB串口 (根据实际情况修改)
    BAUD_RATE = 115200     # USB串口波特率

PACKET_SIZE = 14  # 数据包大小（雷达+Odom）

# 地图参数
MAP_SIZE = 500  # 栅格地图大小 (500x500)
MAP_RESOLUTION = 0.05  # 每个栅格 5cm
MAX_RANGE = 1.5  # 最大有效距离 3.5m（与过滤器一致）

# 🔧 雷达距离校准参数
DISTANCE_SCALE_FACTOR = 0.79    # 距离缩放因子：调整雷达距离与真实世界的比例
                                # > 1.0: 雷达显示距离放大 (点离中心更远)
                                # < 1.0: 雷达显示距离缩小 (点离中心更近)
                                # 例如：1.2 表示雷达距离放大20%

# 🔧 近距离非线性校正参数（修正近距离畸变）
USE_NONLINEAR_CORRECTION = True     # 是否启用非线性距离校正
NEAR_DISTANCE_THRESHOLD = 0.5       # 近距离阈值(米)：小于此距离应用校正
NEAR_CORRECTION_FACTOR = 0.86        # 近距离校正系数：0.85表示近距离缩小15%
                                    # < 1.0: 近距离点向内收缩（修正外凸）
                                    # > 1.0: 近距离点向外扩展
# 平滑过渡参数
CORRECTION_BLEND_RANGE = 0.3      # 校正混合范围(米)：在阈值附近平滑过渡

# 🔧 旋转检测滤波参数（防止旋转时拖尾）
USE_ROTATION_FILTER = True        # 是否启用旋转检测滤波
ROTATION_THRESHOLD_DEG_S = 5.0   # 🔧 角速度阈值(度/秒)：更敏感检测旋转
ROTATION_SMOOTH_WINDOW = 3        # 🔧 平滑窗口：减小以更快响应旋转变化
ROTATION_FILTER_DELAY = 0.3       # 🔧 滤波延迟(秒)：增加延迟避免旋转停止时误建图

# 🛡️ 墙保护参数（防止真实墙衰减过快）
WALL_PROTECTION_ENABLED = True    # 是否启用墙保护机制
HIGH_CONFIDENCE_THRESHOLD = 30   # 高置信度墙阈值（多次观测确认）
HIGH_CONFIDENCE_EROSION = 0.5    # 高置信度墙侵蚀速度（每次-0.5）
NORMAL_EROSION = 1.0             # 普通墙侵蚀速度（每次-1.0）

UPDATE_INTERVAL_MS = 6         # 可视化更新间隔 (ms) - 越小越实时
POINTS_PER_FRAME = 15000       # 每帧处理的数据包数量 - 增加处理量以获取更多点

# Real-time scan显示模式配置（优化以显示更多点）
USE_TIME_WINDOW = True         # True=基于时间窗口, False=基于固定点数
MAX_DISPLAY_TIME = 0.1         # 时间窗口模式：显示最近N秒的点 (500ms = 5圈扫描，更多点)
MAX_DISPLAY_POINTS = 2000      # 固定点数模式：显示最近N个点 (增加到2000点)

USE_MOTION_FILTER = False      # 🚀 禁用运动过滤以显示所有点（更完整的环境显示）
MOTION_THRESHOLD = 0.1         # 运动阈值：点相对机器人移动超过此距离(米)则剔除
ANGLE_THRESHOLD = 10.0         # 角度阈值：机器人旋转超过此角度(度)则剔除点
USE_OVERLAP_PROTECTION = True  # 🧠 重合保护：如果历史点和当前扫描重合，则保留
OVERLAP_DISTANCE = 0.1         # 重合判断距离：点在世界坐标系中距离<10cm视为重合
OVERLAP_RATIO = 0.3            # 重合率阈值：>30%的历史点重合则保护整批数据

USE_MULTITHREADING = True      # 启用多线程优化（需要多核CPU）
USE_SEPARATE_ODOM_THREAD = True  # 🚀 启用独立的Odom更新线程（极致实时性）

# ==================== 数据结构 ====================
class LidarData:
    def __init__(self):
        self.angles = []
        self.distances = []
        self.qualities = []
        self.timestamps = []  # 📍 每个点的接收时间戳
        self.robot_poses = []  # 🚀 每个点被观测时的机器人位姿 (x, y, theta)
        self.scan_count = 0
        
        # 栅格地图（0=未知, 1-100=占用概率, -1=自由空间）
        self.grid_map = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.int8)
        self.robot_pos = (MAP_SIZE // 2, MAP_SIZE // 2)  # 机器人位置（栅格坐标）
        self.robot_theta = 0.0  # 机器人朝向（度）
        self.robot_trajectory = []  # 机器人轨迹（用于显示）

# ==================== 扫描缓冲区（用于位姿插值校正） ====================
class ScanBuffer:
    """
    扫描缓冲区：缓存一圈完整扫描，用于位姿插值校正
    
    核心思想：
    1. 收集一圈扫描的所有点（从SYNC到下一个SYNC）
    2. 记录这一圈的起始位姿和结束位姿
    3. 根据角度进度插值计算每个点扫描时的真实位姿
    4. 用校正后的位姿进行SLAM建图
    """
    def __init__(self):
        self.points = []           # [(angle, distance, quality), ...]
        self.start_pose = None     # (x, y, theta) 起始位姿
        self.end_pose = None       # (x, y, theta) 结束位姿
        self.is_started = False    # 是否已开始收集
    
    def start_new_scan(self, pose):
        """开始新的一圈扫描"""
        self.points = []
        self.start_pose = pose
        self.end_pose = pose
        self.is_started = True
    
    def add_point(self, angle, distance, quality, current_pose):
        """添加点到缓冲区"""
        if self.is_started:
            self.points.append((angle, distance, quality))
            self.end_pose = current_pose  # 持续更新结束位姿
    
    def get_interpolated_pose(self, angle):
        """
        根据角度插值计算位姿
        
        Args:
            angle: 雷达角度 (0-360°)
        
        Returns:
            插值后的位姿 (x, y, theta)
        """
        if not self.is_started or self.start_pose is None:
            return self.end_pose
        
        # 扫描进度：0° = 0%, 360° = 100%
        progress = angle / 360.0
        
        # 线性插值位置
        x = self.start_pose[0] + (self.end_pose[0] - self.start_pose[0]) * progress
        y = self.start_pose[1] + (self.end_pose[1] - self.start_pose[1]) * progress
        
        # 角度插值（处理360°跳变）
        theta_start = self.start_pose[2]
        theta_end = self.end_pose[2]
        delta = theta_end - theta_start
        
        # 处理角度跳变（-180° 和 +180° 之间）
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
        
        theta = theta_start + delta * progress
        
        return (x, y, theta)
    
    def get_motion_distance(self):
        """计算这一圈扫描内机器人的移动距离"""
        if not self.is_started or self.start_pose is None or self.end_pose is None:
            return 0.0
        
        dx = self.end_pose[0] - self.start_pose[0]
        dy = self.end_pose[1] - self.start_pose[1]
        return np.sqrt(dx**2 + dy**2)
    
    def get_rotation_angle(self):
        """计算这一圈扫描内机器人的旋转角度（度）"""
        if not self.is_started or self.start_pose is None or self.end_pose is None:
            return 0.0
        
        delta = self.end_pose[2] - self.start_pose[2]
        
        # 处理角度跳变
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
        
        return abs(delta)

# ==================== 旋转检测滤波类 ====================
class RotationFilter:
    """
    旋转检测滤波：检测机器人旋转状态，防止旋转时建图产生拖尾
    
    原理：
    1. 计算角速度（度/秒）
    2. 超过阈值时暂停建图
    3. 旋转停止后延迟恢复建图
    """
    def __init__(self):
        self.pose_history = []  # 位姿历史 [(timestamp, x, y, theta), ...]
        self.is_rotating = False  # 当前是否在旋转
        self.rotation_stop_time = 0.0  # 旋转停止时间
        self.last_theta = None  # 上次角度（用于角度跳变处理）
    
    def update_pose(self, x, y, theta):
        """更新机器人位姿"""
        import time
        current_time = time.time()
        
        # 处理角度跳变（-180° 和 +180° 之间）
        if self.last_theta is not None:
            delta_theta = theta - self.last_theta
            # 处理角度跳变
            if delta_theta > 180:
                delta_theta -= 360
            elif delta_theta < -180:
                delta_theta += 360
            theta = self.last_theta + delta_theta
        
        # 添加到位姿历史
        self.pose_history.append((current_time, x, y, theta))
        
        # 保持历史长度
        if len(self.pose_history) > ROTATION_SMOOTH_WINDOW:
            self.pose_history.pop(0)
        
        self.last_theta = theta
    
    def is_rotation_detected(self):
        """检测是否在旋转"""
        if not USE_ROTATION_FILTER or len(self.pose_history) < 2:
            return False
        
        # 计算角速度（度/秒）
        current_time, _, _, current_theta = self.pose_history[-1]
        old_time, _, _, old_theta = self.pose_history[0]
        
        time_diff = current_time - old_time
        if time_diff <= 0:
            return False
        
        # 处理角度跳变
        delta_theta = current_theta - old_theta
        if delta_theta > 180:
            delta_theta -= 360
        elif delta_theta < -180:
            delta_theta += 360
        
        angular_velocity = abs(delta_theta) / time_diff  # 度/秒
        
        # 检测旋转状态
        if angular_velocity > ROTATION_THRESHOLD_DEG_S:
            self.is_rotating = True
            self.rotation_stop_time = 0.0  # 重置停止时间
            return True
        else:
            if self.is_rotating:
                # 刚停止旋转，记录停止时间
                if self.rotation_stop_time == 0.0:
                    self.rotation_stop_time = current_time
                self.is_rotating = False
            
            # 检查延迟恢复
            if self.rotation_stop_time > 0:
                if current_time - self.rotation_stop_time > ROTATION_FILTER_DELAY:
                    self.rotation_stop_time = 0.0  # 恢复建图
                    return False
                else:
                    return True  # 仍在延迟期内
            
            return False
    
    def get_status(self):
        """获取滤波状态信息"""
        if not USE_ROTATION_FILTER:
            return "Disabled"
        
        if self.is_rotating:
            return "Rotating"
        elif self.rotation_stop_time > 0:
            import time
            remaining = ROTATION_FILTER_DELAY - (time.time() - self.rotation_stop_time)
            return f"Delay({remaining:.1f}s)"
        else:
            return "Ready"

# ==================== 非线性距离校正函数 ====================
def apply_nonlinear_distance_correction(distance_m):
    """
    应用非线性距离校正，修正近距离畸变
    
    原理：
    - 近距离（< NEAR_DISTANCE_THRESHOLD）：应用校正系数缩小距离
    - 远距离（> NEAR_DISTANCE_THRESHOLD + BLEND_RANGE）：保持原距离
    - 中间范围：平滑过渡（余弦插值）
    
    Args:
        distance_m: 原始距离（米）
    
    Returns:
        校正后的距离（米）
    """
    if not USE_NONLINEAR_CORRECTION:
        return distance_m
    
    # 近距离：直接应用校正系数
    if distance_m < NEAR_DISTANCE_THRESHOLD:
        return distance_m * NEAR_CORRECTION_FACTOR
    
    # 远距离：不校正
    if distance_m > NEAR_DISTANCE_THRESHOLD + CORRECTION_BLEND_RANGE:
        return distance_m
    
    # 中间范围：平滑过渡（余弦插值）
    # t从0（近距离边界）到1（远距离边界）
    t = (distance_m - NEAR_DISTANCE_THRESHOLD) / CORRECTION_BLEND_RANGE
    # 余弦插值：平滑过渡，避免突变
    smooth_t = (1 - np.cos(t * np.pi)) / 2
    
    # 混合近距离校正和原始距离
    corrected_near = distance_m * NEAR_CORRECTION_FACTOR
    corrected = corrected_near * (1 - smooth_t) + distance_m * smooth_t
    
    return corrected

# ==================== 串口数据接收 ====================
class SerialReceiver:
    def __init__(self, port, baudrate, mode="USB"):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.mode = mode
        print(f"✅ Connected to {mode}: {port} @ {baudrate}")
        time.sleep(1)
    
    def diagnose_data(self, duration=3):
        """Diagnose raw data from Bluetooth (for debugging)"""
        print(f"\n🔍 Diagnosing data for {duration} seconds...")
        print("Expected: Binary packets starting with 0xAA55")
        print("Raw data (first 100 bytes):")
        
        start_time = time.time()
        data_buffer = bytearray()
        
        while time.time() - start_time < duration:
            byte = self.ser.read(1)
            if len(byte) > 0:
                data_buffer.extend(byte)
                if len(data_buffer) >= 100:
                    break
        
        if len(data_buffer) == 0:
            print("❌ No data received! Check:")
            print("   1. Is the robot powered on?")
            print("   2. Is Lidar running?")
            print("   3. Is BTSerial sending data?")
            return False
        
        # Display hex dump
        print("\nHex dump:")
        for i in range(0, min(len(data_buffer), 100), 16):
            hex_str = ' '.join([f'{b:02X}' for b in data_buffer[i:i+16]])
            ascii_str = ''.join([chr(b) if 32 <= b < 127 else '.' for b in data_buffer[i:i+16]])
            print(f"  {i:04X}: {hex_str:<48} | {ascii_str}")
        
        # Check for packet headers (little-endian: 0x55 0xAA)
        header_count = 0
        for i in range(len(data_buffer) - 1):
            if data_buffer[i] == 0x55 and data_buffer[i+1] == 0xAA:
                header_count += 1
                print(f"\n✅ Found packet header at offset {i}: 0x55AA (0xAA55 little-endian)")
        
        if header_count == 0:
            print("\n⚠️  No valid packet headers (0xAA55) found!")
            print("This could mean:")
            print("  1. Baud rate mismatch (try 460800 or 115200)")
            print("  2. Robot is sending text debug messages instead of binary")
            print("  3. Data synchronization issue")
            return False
        else:
            print(f"\n✅ Found {header_count} packet header(s)")
            print("Data format looks correct!")
            return True
        
    def read_packet(self):
        """读取一个完整的数据包（支持雷达融合包和独立Odom包）"""
        # 寻找包头 0x55 0xAA (0xAA55的little-endian存储) 或 0x55 0xBB (0xBB55的little-endian存储)
        while True:
            byte1 = self.ser.read(1)
            if len(byte1) == 0:
                return None
                
            if byte1[0] == 0x55:
                byte2 = self.ser.read(1)
                if len(byte2) > 0:
                    # 雷达融合包（0xAA55）
                    if byte2[0] == 0xAA:
                        break  # 找到雷达包头
                    # 独立Odom包（0xBB55）
                    elif byte2[0] == 0xBB:
                        return self._read_odom_only_packet()  # 读取独立Odom包
            
            # 检查同步标记
            if byte1[0] == 0xEE:
                byte2 = self.ser.read(1)
                if len(byte2) > 0 and byte2[0] == 0xEE:
                    return 'SYNC'  # 新的一圈开始
        
        # 读取剩余16字节数据（雷达8字节 + Odom8字节）
        data = self.ser.read(16)
        if len(data) < 16:
            return None
        
        # 解析数据包：angle_q8(4) + distance_mm(2) + quality(1) + odom_x(2) + odom_y(2) + odom_theta(4) + checksum(1)
        # 注意：angle_q8 使用uint32_t (4字节) 以避免360°*256=92160溢出uint16_t
        # 注意：odom_theta_q8 使用int32_t (4字节) 以避免±128°溢出问题
        angle_q8, distance_mm, quality, odom_x_cm, odom_y_cm, odom_theta_q8, checksum = struct.unpack('<IHBhhiB', data)
        
        # 校验（XOR所有字节，除了最后的checksum）
        calc_checksum = 0
        for byte in data[:-1]:
            calc_checksum ^= byte
        
        if calc_checksum != checksum:
            # 校验失败，静默丢弃数据包
            return None
        
        # 转换为实际值
        angle_deg = angle_q8 / 256.0
        # 🔧 应用距离校正：先线性缩放，再非线性校正
        distance_m = distance_mm / 1000.0  # 原始距离（米）
        distance_m = distance_m * DISTANCE_SCALE_FACTOR  # 线性缩放
        distance_m = apply_nonlinear_distance_correction(distance_m)  # 非线性校正（修正近距离畸变）
        odom_x_m = odom_x_cm / 100.0
        odom_y_m = odom_y_cm / 100.0
        odom_theta_deg = odom_theta_q8 / 256.0
        
        return {
            'type': 'LIDAR_ODOM',  # 雷达+Odom融合包
            'angle': angle_deg,
            'distance': distance_m,
            'quality': quality,
            'odom_x': odom_x_m,
            'odom_y': odom_y_m,
            'odom_theta': odom_theta_deg
        }
    
    def _read_odom_only_packet(self):
        """
        读取独立Odom数据包（11字节）
        格式：header(2) + odom_x(2) + odom_y(2) + odom_theta(4) + checksum(1)
        """
        # 读取9字节数据（已读取2字节包头）
        data = self.ser.read(9)
        if len(data) < 9:
            return None
        
        # 解析数据包：odom_x(2) + odom_y(2) + odom_theta(4) + checksum(1)
        odom_x_cm, odom_y_cm, odom_theta_q8, checksum = struct.unpack('<hhiB', data)
        
        # 校验（XOR所有字节，除了最后的checksum）
        calc_checksum = 0
        for byte in data[:-1]:
            calc_checksum ^= byte
        
        if calc_checksum != checksum:
            # 校验失败，静默丢弃数据包
            return None
        
        # 转换为实际值
        odom_x_m = odom_x_cm / 100.0
        odom_y_m = odom_y_cm / 100.0
        odom_theta_deg = odom_theta_q8 / 256.0
        
        return {
            'type': 'ODOM_ONLY',  # 独立Odom包
            'odom_x': odom_x_m,
            'odom_y': odom_y_m,
            'odom_theta': odom_theta_deg
        }

# ==================== 共享Odom数据（线程安全） ====================
class SharedOdomData:
    """
    线程安全的Odom数据共享容器
    - DataReceiverThread写入最新Odom
    - 主线程和其他线程读取
    """
    def __init__(self):
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.timestamp = 0.0
        self.lock = threading.Lock()
        self.update_count = 0
    
    def update(self, x, y, theta):
        """更新Odom数据（线程安全）"""
        with self.lock:
            self.robot_x = x
            self.robot_y = y
            self.robot_theta = theta
            self.timestamp = time.time()
            self.update_count += 1
    
    def get(self):
        """获取Odom数据（线程安全）"""
        with self.lock:
            return (self.robot_x, self.robot_y, self.robot_theta)

# ==================== 多线程数据接收器 ====================
class DataReceiverThread(threading.Thread):
    """
    独立的数据接收线程，持续从串口读取数据并放入队列
    优势：
    1. 数据接收不会被可视化更新阻塞
    2. 可视化更新不会被串口读取阻塞
    3. 充分利用多核CPU性能
    4. 🚀 实时更新SharedOdomData（不经过队列，零延迟）
    """
    def __init__(self, serial_receiver, data_queue, shared_odom=None, max_queue_size=10000):
        super().__init__(daemon=True)  # 设为守护线程，主程序退出时自动结束
        self.serial = serial_receiver
        self.data_queue = data_queue
        self.shared_odom = shared_odom  # 🚀 共享Odom数据
        self.max_queue_size = max_queue_size
        self.running = True
        self.packet_count = 0
        self.drop_count = 0
        self.odom_only_count = 0  # 🚀 独立Odom包计数
        self.lidar_odom_count = 0  # 雷达+Odom融合包计数
        
    def run(self):
        """线程主循环：持续读取串口数据"""
        print(f"🧵 Data receiver thread started (PID: {threading.get_ident()})")
        
        while self.running:
            try:
                packet = self.serial.read_packet()
                
                if packet is not None:
                    self.packet_count += 1
                    
                    # 统计不同类型的包
                    if isinstance(packet, dict):
                        pkt_type = packet.get('type')
                        if pkt_type == 'ODOM_ONLY':
                            self.odom_only_count += 1
                        elif pkt_type == 'LIDAR_ODOM':
                            self.lidar_odom_count += 1
                    
                    # 🚀 立即更新SharedOdomData（最高优先级，零延迟）
                    # 优先使用独立Odom包（更实时），其次使用融合包
                    if self.shared_odom is not None and packet != 'SYNC' and isinstance(packet, dict):
                        robot_x = packet.get('odom_x')
                        robot_y = packet.get('odom_y')
                        robot_theta = packet.get('odom_theta')
                        if robot_x is not None:
                            self.shared_odom.update(robot_x, robot_y, robot_theta)
                    
                    # 如果队列已满，丢弃最旧的数据
                    if self.data_queue.qsize() >= self.max_queue_size:
                        try:
                            self.data_queue.get_nowait()  # 移除最旧的数据
                            self.drop_count += 1
                        except queue.Empty:
                            pass
                    
                    # 将新数据放入队列（非阻塞）
                    try:
                        self.data_queue.put_nowait(packet)
                    except queue.Full:
                        self.drop_count += 1
                        
            except Exception as e:
                print(f"⚠️  Receiver thread error: {e}")
                time.sleep(0.001)  # 出错时短暂休眠
        
        print(f"🧵 Data receiver thread stopped (Total: {self.packet_count}, "
              f"Odom: {self.odom_only_count}, Lidar: {self.lidar_odom_count}, Dropped: {self.drop_count})")
    
    def stop(self):
        """停止线程"""
        self.running = False

# ==================== 地图保存/加载管理 ====================
class MapManager:
    """地图保存和加载管理器"""
    
    @staticmethod
    def save_map(grid_map, robot_trajectory, metadata=None, filename_prefix='maze_map'):
        """
        保存地图（支持多种格式）
        
        Args:
            grid_map: 栅格地图数组
            robot_trajectory: 机器人轨迹列表
            metadata: 元数据字典
            filename_prefix: 文件名前缀
        """
        # 创建保存目录
        save_dir = 'saved_maps'
        os.makedirs(save_dir, exist_ok=True)
        
        # 生成时间戳文件名
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        base_filename = f"{filename_prefix}_{timestamp}"
        
        # 1. 保存为NumPy数组（.npy，精确数据）
        npy_file = os.path.join(save_dir, f"{base_filename}_map.npy")
        np.save(npy_file, grid_map)
        print(f"✅ 地图数据已保存: {npy_file}")
        
        # 2. 保存为PNG图像（可视化）
        png_file = os.path.join(save_dir, f"{base_filename}_map.png")
        MapManager._save_as_image(grid_map, robot_trajectory, png_file)
        print(f"✅ 地图图像已保存: {png_file}")
        
        # 3. 保存为JSON（包含元数据）
        json_file = os.path.join(save_dir, f"{base_filename}_metadata.json")
        map_data = {
            'timestamp': timestamp,
            'map_size': grid_map.shape,
            'resolution': MAP_RESOLUTION,
            'max_range': MAX_RANGE,
            'robot_trajectory': robot_trajectory,
            'npy_file': npy_file,
            'png_file': png_file
        }
        if metadata:
            map_data.update(metadata)
        
        with open(json_file, 'w', encoding='utf-8') as f:
            json.dump(map_data, f, indent=2, ensure_ascii=False)
        print(f"✅ 元数据已保存: {json_file}")
        
        print(f"\n📁 地图保存完成！位置: {save_dir}/")
        return base_filename
    
    @staticmethod
    def _save_as_image(grid_map, robot_trajectory, filename):
        """将地图保存为PNG图像"""
        # 创建RGB图像
        img_array = np.zeros((grid_map.shape[0], grid_map.shape[1], 3), dtype=np.uint8)
        
        # 颜色映射（改进版：障碍物更深更清晰）
        # 未知区域(0) -> 灰色(200)
        # 自由空间(-10~-1) -> 白色(255)
        # 障碍物(1-80) -> 深灰到纯黑(深色)
        
        for i in range(grid_map.shape[0]):
            for j in range(grid_map.shape[1]):
                value = grid_map[i, j]
                if value == 0:  # 未知
                    img_array[i, j] = [200, 200, 200]  # 浅灰
                elif value < 0:  # 自由空间 (-10~-1)
                    img_array[i, j] = [255, 255, 255]  # 白色
                else:  # 障碍物 (1-80)
                    # 非线性映射：让障碍物更黑
                    # 1-20: 150-50 (快速变黑)
                    # 20-80: 50-0 (纯黑)
                    if value < 20:
                        intensity = int(150 - value * 5)  # 1->145, 20->50
                    else:
                        intensity = max(0, int(50 - (value - 20) * 0.83))  # 20->50, 80->0
                    img_array[i, j] = [intensity, intensity, intensity]
        
        # 绘制机器人轨迹（蓝色）
        for i in range(len(robot_trajectory) - 1):
            x0, y0 = robot_trajectory[i]
            x1, y1 = robot_trajectory[i + 1]
            # 简单线段绘制
            if 0 <= x0 < grid_map.shape[1] and 0 <= y0 < grid_map.shape[0]:
                img_array[int(y0), int(x0)] = [0, 0, 255]
        
        # 标记起点（绿色）和终点（红色）
        if len(robot_trajectory) > 0:
            start_x, start_y = robot_trajectory[0]
            if 0 <= start_x < grid_map.shape[1] and 0 <= start_y < grid_map.shape[0]:
                for dy in range(-2, 3):
                    for dx in range(-2, 3):
                        sy, sx = int(start_y + dy), int(start_x + dx)
                        if 0 <= sx < grid_map.shape[1] and 0 <= sy < grid_map.shape[0]:
                            img_array[sy, sx] = [0, 255, 0]  # 绿色起点
            
            end_x, end_y = robot_trajectory[-1]
            if 0 <= end_x < grid_map.shape[1] and 0 <= end_y < grid_map.shape[0]:
                for dy in range(-2, 3):
                    for dx in range(-2, 3):
                        ey, ex = int(end_y + dy), int(end_x + dx)
                        if 0 <= ex < grid_map.shape[1] and 0 <= ey < grid_map.shape[0]:
                            img_array[ey, ex] = [255, 0, 0]  # 红色终点
        
        # 翻转Y轴（图像坐标系与栅格坐标系不同）
        img_array = np.flipud(img_array)
        
        # 保存为PNG
        img = Image.fromarray(img_array)
        img.save(filename)
    
    @staticmethod
    def load_map(filename):
        """
        加载地图
        
        Args:
            filename: .npy文件路径或JSON文件路径
        
        Returns:
            (grid_map, metadata) 元组
        """
        if filename.endswith('.json'):
            # 从JSON加载
            with open(filename, 'r', encoding='utf-8') as f:
                metadata = json.load(f)
            grid_map = np.load(metadata['npy_file'])
            print(f"✅ 地图已加载: {filename}")
            return grid_map, metadata
        
        elif filename.endswith('.npy'):
            # 直接加载NumPy数组
            grid_map = np.load(filename)
            print(f"✅ 地图已加载: {filename}")
            return grid_map, None
        
        else:
            raise ValueError("不支持的文件格式！请使用.npy或.json文件")
    
    @staticmethod
    def list_saved_maps(save_dir='saved_maps'):
        """列出所有已保存的地图"""
        if not os.path.exists(save_dir):
            print("📂 还没有保存过地图")
            return []
        
        json_files = [f for f in os.listdir(save_dir) if f.endswith('_metadata.json')]
        
        if len(json_files) == 0:
            print("📂 还没有保存过地图")
            return []
        
        print(f"\n📂 已保存的地图 ({len(json_files)}个):")
        for i, json_file in enumerate(sorted(json_files, reverse=True)):
            json_path = os.path.join(save_dir, json_file)
            with open(json_path, 'r', encoding='utf-8') as f:
                metadata = json.load(f)
            print(f"  [{i+1}] {metadata['timestamp']} - {json_file}")
        
        return [os.path.join(save_dir, f) for f in sorted(json_files, reverse=True)]

# ==================== 简单栅格地图SLAM ====================
class SimpleGridSLAM:
    def __init__(self, lidar_data):
        self.data = lidar_data
        
        # ========== 坐标系配置 ==========
        # RPLidar C1: 顺时针旋转为正（左手系）
        # IM948 IMU: 逆时针旋转为正（右手系）
        # 需要将雷达角度镜像转换为逆时针系统
        self.LIDAR_CLOCKWISE = True    # RPLidar C1是顺时针
        self.IMU_CLOCKWISE = False     # IMU是右手系
        self.LIDAR_ANGLE_OFFSET = 0    # 角度零点偏移（如果需要可以调整：90, 180, 270）
        self.DEBUG_MODE = False        # 调试模式（根据需要开启）
        # ================================
        
        # 🔧 旋转检测滤波
        self.rotation_filter = RotationFilter()
        
    def update_map(self, angle_deg, distance_m, robot_x, robot_y, robot_theta, weight=8, skip_rotation_check=False):
        """
        动态SLAM：考虑机器人位姿的地图更新
        
        Args:
            weight: 障碍物累积权重（默认8，实时建图可用更小值如3）
            skip_rotation_check: 是否跳过旋转检测（如果外部已经检测过，避免重复）
        """
        # 🔧 旋转检测滤波：旋转时不建图，防止拖尾
        # 注意：如果外部已经检测过旋转状态，则跳过此检查
        if not skip_rotation_check:
            # 更新旋转检测滤波
            self.rotation_filter.update_pose(robot_x, robot_y, robot_theta)
            
            # 检测旋转状态
            if self.rotation_filter.is_rotation_detected():
                if self.DEBUG_MODE:
                    print(f"🔄 Rotation detected, skipping mapping (Status: {self.rotation_filter.get_status()})")
                return
        
        # 🔧 建图距离过滤：丢弃20cm以内和超出MAX_RANGE的点
        MIN_MAP_DISTANCE = 0.20  # 最小建图距离：20cm
        if distance_m <= MIN_MAP_DISTANCE or distance_m > MAX_RANGE:
            return
        
        # ========== 坐标系转换 ==========
        # 保存原始值用于调试
        angle_orig = angle_deg
        theta_orig = robot_theta
        
        # 1. 修正雷达角度（RPLidar C1是顺时针，转换为逆时针）
        if self.LIDAR_CLOCKWISE:
            angle_deg = 360.0 - angle_deg  # 顺时针 → 逆时针镜像
            if angle_deg >= 360.0:
                angle_deg -= 360.0
        
        # 2. 应用雷达角度偏移（如果雷达安装方向有偏移）
        angle_deg = (angle_deg + self.LIDAR_ANGLE_OFFSET) % 360.0
        
        # 3. 修正IMU角度（如果IMU也是顺时针，需要镜像）
        if self.IMU_CLOCKWISE:
            robot_theta = -robot_theta
        
        # 调试输出（监控位姿和角度转换）
        if self.DEBUG_MODE:
            if not hasattr(self, '_debug_counter'):
                self._debug_counter = 0
                self._last_robot_pos = (robot_x, robot_y, robot_theta)
            
            self._debug_counter += 1
            if self._debug_counter % 100 == 0:  # 每100个点打印一次
                dx = robot_x - self._last_robot_pos[0]
                dy = robot_y - self._last_robot_pos[1]
                dtheta = robot_theta - self._last_robot_pos[2]
                print(f"🤖 Robot: Pos=({robot_x:.3f}, {robot_y:.3f})m θ={robot_theta:.1f}° | "
                      f"ΔPos=({dx:.3f}, {dy:.3f})m Δθ={dtheta:.1f}° | "
                      f"Lidar: {angle_orig:.0f}°→{angle_deg:.0f}° dist={distance_m:.2f}m")
                self._last_robot_pos = (robot_x, robot_y, robot_theta)
        # ===============================
        
        # 将机器人世界坐标转换为地图栅格坐标
        # 应用显示旋转变换：逆时针90° (x_display = -y, y_display = x)
        rx_grid = int(MAP_SIZE / 2 + (-robot_y) / MAP_RESOLUTION)
        ry_grid = int(MAP_SIZE / 2 + robot_x / MAP_RESOLUTION)
        
        # 计算障碍物在世界坐标系中的位置
        # 1. 雷达测量角度 + 机器人朝向
        world_angle_rad = np.deg2rad(angle_deg + robot_theta)
        
        # 2. 障碍物世界坐标
        obstacle_world_x = robot_x + distance_m * np.cos(world_angle_rad)
        obstacle_world_y = robot_y + distance_m * np.sin(world_angle_rad)
        
        # 3. 转换为地图栅格坐标（应用显示旋转）
        ox_grid = int(MAP_SIZE / 2 + (-obstacle_world_y) / MAP_RESOLUTION)
        oy_grid = int(MAP_SIZE / 2 + obstacle_world_x / MAP_RESOLUTION)
        
        # 检查边界
        if not (0 <= rx_grid < MAP_SIZE and 0 <= ry_grid < MAP_SIZE):
            return
        
        # Bresenham画线算法：标记自由空间
        self._bresenham_line(rx_grid, ry_grid, ox_grid, oy_grid)
        
        # 标记障碍物（概率累积，适配快速移动）
        if 0 <= ox_grid < MAP_SIZE and 0 <= oy_grid < MAP_SIZE:
            # 使用可配置权重：实时建图用较小权重(3)，批量建图用正常权重(8)
            # 上限设为60，提供更强的"惯性"抵抗自由空间侵蚀
            self.data.grid_map[oy_grid, ox_grid] = min(60, self.data.grid_map[oy_grid, ox_grid] + weight)
        
        # 更新机器人当前位置和朝向（用于显示）
        self.data.robot_pos = (rx_grid, ry_grid)
        self.data.robot_theta = robot_theta  # 保存朝向角度
        
        # 记录轨迹（降低阈值以提高刷新率：从5改为1）
        if len(self.data.robot_trajectory) == 0 or \
           (abs(self.data.robot_trajectory[-1][0] - rx_grid) + abs(self.data.robot_trajectory[-1][1] - ry_grid) > 2):
            self.data.robot_trajectory.append((rx_grid, ry_grid))
    
    def _bresenham_line(self, x0, y0, x1, y1):
        """Bresenham直线算法 - 标记自由空间"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        while True:
            # 改进的自由空间标记：平衡清除速度（适配快速移动）
            if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE:
                current_val = self.data.grid_map[y, x]
                if current_val > 0:
                    # 如果当前是障碍物，根据置信度分级侵蚀
                    if WALL_PROTECTION_ENABLED and current_val > HIGH_CONFIDENCE_THRESHOLD:
                        # 高置信度墙侵蚀更慢（保护真实墙）
                        self.data.grid_map[y, x] = max(-10, current_val - HIGH_CONFIDENCE_EROSION)
                    else:  # 低置信度墙或墙保护关闭
                        # 普通侵蚀速度
                        self.data.grid_map[y, x] = max(-10, current_val - NORMAL_EROSION)
                else:
                    # 如果是未知或已是自由空间，继续降低（最低到-10）
                    self.data.grid_map[y, x] = max(-10, current_val - 1)
            
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

# ==================== 可视化 ====================
class LidarVisualizer:
    def __init__(self, serial_receiver, use_multithreading=False):
        self.serial = serial_receiver
        self.lidar_data = LidarData()
        self.slam = SimpleGridSLAM(self.lidar_data)
        
        # 🚀 扫描缓冲区（用于位姿插值校正）
        self.scan_buffer = ScanBuffer()
        
        # 🚀 共享Odom数据（线程安全，零延迟更新）
        self.shared_odom = SharedOdomData() if USE_SEPARATE_ODOM_THREAD else None
        
        # 多线程模式配置
        self.use_multithreading = use_multithreading
        if self.use_multithreading:
            self.data_queue = queue.Queue(maxsize=20000)  # 大容量队列
            self.receiver_thread = DataReceiverThread(
                serial_receiver, 
                self.data_queue,
                shared_odom=self.shared_odom  # 传递共享Odom
            )
            self.receiver_thread.start()
            if USE_SEPARATE_ODOM_THREAD:
                print(f"✅ Multi-threading enabled: Receiver thread + 🚀 Zero-latency Odom update")
            else:
                print(f"✅ Multi-threading enabled: Receiver thread running on separate core")
        
        # 创建图形（调整尺寸为原来的75%）
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 5.25))
        
        # Left: Real-time point cloud
        self.ax1.set_xlim(-MAX_RANGE, MAX_RANGE)
        self.ax1.set_ylim(-MAX_RANGE, MAX_RANGE)
        self.ax1.set_aspect('equal')
        
        # 🔧 设置更细的网格：每格10cm (0.1m)
        # 主刻度：每0.5m显示数字标签（稀疏显示）
        major_label_ticks = np.arange(-MAX_RANGE, MAX_RANGE + 0.5, 0.5)
        self.ax1.set_xticks(major_label_ticks)
        self.ax1.set_yticks(major_label_ticks)
        
        # 次刻度：每0.1m（10cm）显示网格线但无标签
        minor_grid_ticks = np.arange(-MAX_RANGE, MAX_RANGE + 0.1, 0.1)
        self.ax1.set_xticks(minor_grid_ticks, minor=True)
        self.ax1.set_yticks(minor_grid_ticks, minor=True)
        
        # 更细的次刻度：每0.05m（5cm）显示更细网格线
        fine_grid_ticks = np.arange(-MAX_RANGE, MAX_RANGE + 0.05, 0.05)
        
        # 网格样式设置
        # 主网格（0.5m间隔）：浅灰色，稍粗
        self.ax1.grid(True, which='major', alpha=0.5, linewidth=0.8, color='lightgray')
        # 次网格（0.1m间隔）：浅灰色，细线
        self.ax1.grid(True, which='minor', alpha=0.3, linewidth=0.4, color='lightgray')
        
        # 手动添加更细的5cm网格线
        for x in fine_grid_ticks:
            if x not in minor_grid_ticks:  # 避免重复
                self.ax1.axvline(x, alpha=0.15, linewidth=0.2, color='lightgray', zorder=0)
        for y in fine_grid_ticks:
            if y not in minor_grid_ticks:  # 避免重复
                self.ax1.axhline(y, alpha=0.15, linewidth=0.2, color='lightgray', zorder=0)
        
        self.ax1.set_title('Real-time Lidar Scan (Grid: 10cm, Labels: 0.5m)', fontsize=12, pad=10)
        self.ax1.set_xlabel('X (m)', fontsize=10)
        self.ax1.set_ylabel('Y (m)', fontsize=10)
        self.scatter = self.ax1.scatter([], [], s=2, c='blue', alpha=0.6, zorder=15)  # 设置最高图层
        
        # 🔴 机器人位置标记（原点红色亮点）
        self.origin_marker = self.ax1.scatter([0], [0], s=80, c='red', marker='o', 
                                             edgecolors='white', linewidths=2, 
                                             alpha=0.9, zorder=20, label='Robot')
        # 添加十字线标记原点
        self.ax1.axhline(0, color='red', linewidth=1.5, alpha=0.5, linestyle='-', zorder=19)
        self.ax1.axvline(0, color='red', linewidth=1.5, alpha=0.5, linestyle='-', zorder=19)
        self.ax1.legend(loc='upper right', fontsize=9)
        
        # Right: Grid map + Robot trajectory（聚焦到中心区域，减少空白）
        # 显示中心±150格（3米范围，对应±150格 @ 0.05m/格）
        map_center = MAP_SIZE // 2  # 250
        map_view_range = 150  # 显示±150格 = ±7.5m范围
        
        self.ax2.set_title('SLAM Grid Map', fontsize=12, pad=10)
        self.ax2.set_xlim(map_center - map_view_range, map_center + map_view_range)
        self.ax2.set_ylim(map_center - map_view_range, map_center + map_view_range)
        self.ax2.set_aspect('equal')
        self.ax2.set_xlabel('Grid X', fontsize=10)
        self.ax2.set_ylabel('Grid Y', fontsize=10)
        
        self.map_img = self.ax2.imshow(
            self.lidar_data.grid_map, 
            cmap='gray_r', 
            origin='lower',
            vmin=-10,  # 自由空间下限（更强的自由空间证据）
            vmax=30,   # 降低上限让障碍物更快变黑（原80太高导致浅灰）
            extent=[0, MAP_SIZE, 0, MAP_SIZE],  # 设置坐标范围
            zorder=1  # 地图图层在底部
        )
        
        # Robot trajectory line
        self.trajectory_line, = self.ax2.plot([], [], 'b-', linewidth=1.5, alpha=0.7, label='Path', zorder=10)
        
        # Robot current position with orientation (dashed ray line extending to map edge)
        # 初始化方向射线（红色虚线，从机器人位置延伸到地图边缘）
        self.robot_direction_line, = self.ax2.plot(
            [map_center, map_center + 50], [map_center, map_center],  # 初始位置和方向
            'r--',  # 红色虚线
            linewidth=2,  # 线宽
            label='Robot Direction',
            zorder=12  # 提高图层，显示在地图上方
        )
        
        # 机器人中心点（红色圆点）
        self.robot_center, = self.ax2.plot(
            [map_center], [map_center],
            'ro',  # 红色圆点
            markersize=8,
            label='Robot',
            zorder=15  # 最高图层，确保可见
        )
        
        self.ax2.legend(loc='upper right', fontsize=9)
        self.ax2.grid(True, alpha=0.2)
        
        # 状态栏文本（图幅上方横向展开）
        self.status_text = self.fig.text(
            0.5, 0.96, '',  # 中央上方位置
            ha='center',  # 居中对齐
            va='top',  # 顶部对齐
            fontsize=9,  # 字体
            color='black',  # 黑色文字
            weight='bold',  # 粗体
            family='monospace',  # 等宽字体，数字对齐更整齐
            bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.85, edgecolor='orange', linewidth=2)  # 黄色半透明背景框
        )
        
        # 帮助文本（底部左侧）
        help_text = "Distance(Left): [+][−][0] | Control: [↑↓←→]Move [Space]Stop | NearCorrect(Right): [◀][▶][N] | Map: [S]Save [Q]Quit"
        self.fig.text(0.01, 0.01, help_text, ha='left', va='bottom', fontsize=7, color='blue')
        
        # === 精准控制面板（底部中央横向排列） ===
        bottom_y = 0.04  # 底部位置
        button_height = 0.03
        button_width = 0.04
        spacing = 0.005
        
        # 标题
        self.fig.text(0.5, 0.085, '━━━━ Precise Control | Distance Calibration | Near Correction ━━━━', ha='center', fontsize=8, 
                     weight='bold', color='darkblue')
        
        # 🎨 对称布局：距离校准按钮(左) ←→ 近距离校正按钮(右) 关于中心对称
        # 左侧：[+][−][0] 距离校准按钮
        dist_start_x = 0.08  # 左侧起始位置
        ax_dist_plus = plt.axes([dist_start_x, bottom_y, button_width, button_height])
        ax_dist_minus = plt.axes([dist_start_x + (button_width + spacing), bottom_y, button_width, button_height])
        ax_dist_reset = plt.axes([dist_start_x + (button_width + spacing) * 2, bottom_y, button_width, button_height])
        
        self.btn_dist_plus = Button(ax_dist_plus, '+', color='lightcoral', hovercolor='red')
        self.btn_dist_minus = Button(ax_dist_minus, '−', color='lightcyan', hovercolor='cyan')
        self.btn_dist_reset = Button(ax_dist_reset, '0', color='lightgray', hovercolor='gray')
        
        # 距离校准按钮事件绑定
        self.btn_dist_plus.label.set_fontsize(10)
        self.btn_dist_plus.on_clicked(lambda event: self.adjust_distance_scale(0.01))
        
        self.btn_dist_minus.label.set_fontsize(10)
        self.btn_dist_minus.on_clicked(lambda event: self.adjust_distance_scale(-0.01))
        
        self.btn_dist_reset.label.set_fontsize(9)
        self.btn_dist_reset.on_clicked(lambda event: self.reset_distance_scale())
        
        # 中间：控制面板（输入框 + 运动按钮）
        start_x = 0.23  # 中间偏左起始位置
        
        # 输入框（指令输入）
        ax_textbox = plt.axes([start_x, bottom_y, 0.1, button_height])
        self.textbox = TextBox(ax_textbox, '', initial='', color='lightyellow', hovercolor='yellow')
        self.textbox.label.set_fontsize(8)
        self.textbox.on_submit(self.on_textbox_submit)
        
        # 发送按钮
        ax_send_btn = plt.axes([start_x + 0.1 + spacing, bottom_y, button_width, button_height])
        self.send_btn = Button(ax_send_btn, 'Send', color='lightgreen', hovercolor='green')
        self.send_btn.label.set_fontsize(8)
        self.send_btn.on_clicked(self.on_send_command)
        
        # 前进距离快捷按钮（横向排列）
        btn_start_x = start_x + 0.1 + button_width + spacing * 3
        ax_f10 = plt.axes([btn_start_x, bottom_y, button_width, button_height])
        ax_f30 = plt.axes([btn_start_x + (button_width + spacing), bottom_y, button_width, button_height])
        ax_f50 = plt.axes([btn_start_x + (button_width + spacing) * 2, bottom_y, button_width, button_height])
        ax_f100 = plt.axes([btn_start_x + (button_width + spacing) * 3, bottom_y, button_width, button_height])
        
        self.btn_f10 = Button(ax_f10, 'F10', color='lightblue', hovercolor='blue')
        self.btn_f30 = Button(ax_f30, 'F30', color='lightblue', hovercolor='blue')
        self.btn_f50 = Button(ax_f50, 'F50', color='lightblue', hovercolor='blue')
        self.btn_f100 = Button(ax_f100, 'F100', color='lightblue', hovercolor='blue')
        
        for btn, label in [(self.btn_f10, 'F10'), (self.btn_f30, 'F30'), 
                          (self.btn_f50, 'F50'), (self.btn_f100, 'F100')]:
            btn.label.set_fontsize(7)
            btn.on_clicked(lambda event, cmd=label: self.send_precise_command(cmd))
        
        # 转向快捷按钮（横向排列）
        turn_start_x = btn_start_x + (button_width + spacing) * 4 + spacing * 2
        ax_l90 = plt.axes([turn_start_x, bottom_y, button_width, button_height])
        ax_l45 = plt.axes([turn_start_x + (button_width + spacing), bottom_y, button_width, button_height])
        ax_r45 = plt.axes([turn_start_x + (button_width + spacing) * 2, bottom_y, button_width, button_height])
        ax_r90 = plt.axes([turn_start_x + (button_width + spacing) * 3, bottom_y, button_width, button_height])
        
        self.btn_l90 = Button(ax_l90, 'L90', color='lightyellow', hovercolor='orange')
        self.btn_l45 = Button(ax_l45, 'L45', color='lightyellow', hovercolor='orange')
        self.btn_r45 = Button(ax_r45, 'R45', color='lightyellow', hovercolor='orange')
        self.btn_r90 = Button(ax_r90, 'R90', color='lightyellow', hovercolor='orange')
        
        for btn, label in [(self.btn_l90, 'L90'), (self.btn_l45, 'L45'), 
                          (self.btn_r45, 'R45'), (self.btn_r90, 'R90')]:
            btn.label.set_fontsize(7)
            btn.on_clicked(lambda event, cmd=label: self.send_precise_command(cmd))
        
        # 右侧：[◀][▶][N] 近距离校正按钮（与左侧距离校准对称）
        # 计算对称位置：左侧3个按钮占 0.08~0.21，右侧应为 0.79~0.92（关于0.5对称）
        near_start_x = 0.79  # 右侧位置（与左侧距离校准按钮关于窗口中心完全对称）
        ax_near_minus = plt.axes([near_start_x, bottom_y, button_width, button_height])
        ax_near_plus = plt.axes([near_start_x + (button_width + spacing), bottom_y, button_width, button_height])
        ax_near_toggle = plt.axes([near_start_x + (button_width + spacing) * 2, bottom_y, button_width, button_height])
        
        self.btn_near_minus = Button(ax_near_minus, '◀', color='lightpink', hovercolor='pink')
        self.btn_near_plus = Button(ax_near_plus, '▶', color='lightgreen', hovercolor='green')
        self.btn_near_toggle = Button(ax_near_toggle, 'N', color='lightyellow', hovercolor='yellow')
        
        # 近距离校正按钮事件绑定
        self.btn_near_minus.label.set_fontsize(9)
        self.btn_near_minus.on_clicked(lambda event: self.adjust_near_correction(-0.02))
        
        self.btn_near_plus.label.set_fontsize(9)
        self.btn_near_plus.on_clicked(lambda event: self.adjust_near_correction(0.02))
        
        self.btn_near_toggle.label.set_fontsize(9)
        self.btn_near_toggle.on_clicked(lambda event: self.toggle_nonlinear_correction())
        
        # 绑定键盘事件
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.fig.canvas.mpl_connect('key_release_event', self.on_key_release)
        
        # 记录当前按下的方向键
        self.current_direction_key = None
        
        # 调整布局，为顶部和底部控制面板留出空间
        plt.tight_layout(rect=[0, 0.10, 1, 0.94])
    
    def process_scan_buffer(self):
        """
        处理完整的一圈扫描：位姿插值 + 批量建图
        
        核心思想：
        1. 检查扫描运动量（过大则丢弃）
        2. 对每个点根据角度进度插值计算真实位姿
        3. 用校正后的位姿批量建图
        """
        if len(self.scan_buffer.points) == 0:
            return
        
        # 计算扫描内运动量
        motion_dist = self.scan_buffer.get_motion_distance()
        rotation_angle = self.scan_buffer.get_rotation_angle()
        
        # 🚀 运动过滤：如果一圈内运动过大，说明位姿不可靠
        # RPLidar C1: 10Hz = 100ms/圈
        # 合理速度：0.5m/s → 100ms内移动5cm
        # 合理旋转：90°/s → 100ms内旋转9°
        MAX_SCAN_MOTION = 0.15      # 100ms内移动>15cm = 1.5m/s，过快
        MAX_SCAN_ROTATION = 8.0     # 🔧 降低旋转阈值：100ms内旋转>8° = 80°/s，与实时旋转检测一致
        
        if motion_dist > MAX_SCAN_MOTION:
            if self.slam.DEBUG_MODE:
                print(f"⚠️  Scan motion too large: {motion_dist*1000:.1f}mm (>{MAX_SCAN_MOTION*1000:.0f}mm), skipping...")
            return
        
        if rotation_angle > MAX_SCAN_ROTATION:
            if self.slam.DEBUG_MODE:
                print(f"⚠️  Scan rotation too large: {rotation_angle:.1f}° (>{MAX_SCAN_ROTATION:.0f}°), skipping...")
            return
        
        # 对每个点进行位姿插值校正并建图
        corrected_count = 0
        for angle, distance, quality in self.scan_buffer.points:
            # 🎯 根据角度插值计算该点扫描时的真实位姿
            interp_pose = self.scan_buffer.get_interpolated_pose(angle)
            
            if interp_pose is not None:
                # 用校正后的位姿更新地图
                # skip_rotation_check=True 因为已经在函数开始时检查了整圈扫描的旋转状态
                self.slam.update_map(angle, distance, 
                                    interp_pose[0], interp_pose[1], interp_pose[2],
                                    skip_rotation_check=True)
                corrected_count += 1
        
        # 调试信息
        if self.slam.DEBUG_MODE and corrected_count > 0:
            print(f"✅ Processed scan: {corrected_count} points, "
                  f"motion={motion_dist*1000:.1f}mm, rotation={rotation_angle:.1f}°")
        
    def update(self, frame):
        """
        动画更新函数（优化版：位置更新优先于建图）
        
        策略：
        0. 【最高优先级】从SharedOdomData立即更新位置（零延迟）
        1. 第一阶段：快速读取所有数据包（建图数据）
        2. 第二阶段：批量处理建图操作（低优先级）
        """
        current_odom = None
        pending_map_updates = []  # 待建图的点列表
        need_process_scan = False  # 是否需要处理扫描缓冲区
        
        # ========== 【最高优先级】立即更新Odom（从共享数据，零延迟） ==========
        if USE_SEPARATE_ODOM_THREAD and self.shared_odom is not None:
            robot_x, robot_y, robot_theta = self.shared_odom.get()
            current_odom = (robot_x, robot_y, robot_theta)
            
            # 🚀 立即更新机器人位姿显示
            self.lidar_data.robot_theta = robot_theta
            rx_grid = int(MAP_SIZE / 2 + (-robot_y) / MAP_RESOLUTION)
            ry_grid = int(MAP_SIZE / 2 + robot_x / MAP_RESOLUTION)
            self.lidar_data.robot_pos = (rx_grid, ry_grid)
        
        # ========== 第一阶段：快速读取数据包（建图数据） ==========
        # 🚀 实时旋转检测：在收集数据前先更新旋转状态
        if current_odom is not None:
            robot_x, robot_y, robot_theta = current_odom
            self.slam.rotation_filter.update_pose(robot_x, robot_y, robot_theta)
        
        for _ in range(POINTS_PER_FRAME):
            # 多线程模式：从队列读取数据
            if self.use_multithreading:
                try:
                    packet = self.data_queue.get_nowait()  # 非阻塞读取
                except queue.Empty:
                    break  # 队列为空，退出循环
            # 单线程模式：直接从串口读取
            else:
                packet = self.serial.read_packet()
            
            if packet is None:
                continue
                
            if packet == 'SYNC':
                # 🚀 标记需要处理扫描缓冲区（但不立即处理，避免阻塞位置更新）
                if current_odom is not None:
                    need_process_scan = True
                
                self.lidar_data.scan_count += 1
                continue
            
            # 🚀 处理独立Odom包（只更新位姿，不添加雷达点）
            if isinstance(packet, dict) and packet.get('type') == 'ODOM_ONLY':
                # 独立Odom包已经在接收线程中更新SharedOdomData
                # 这里无需额外处理，直接跳过
                continue
            
            # 添加雷达数据点（只处理LIDAR_ODOM类型的包）
            if not isinstance(packet, dict) or packet.get('type') != 'LIDAR_ODOM':
                continue
            
            angle = packet['angle']
            distance = packet['distance']
            quality = packet['quality']
            
            # 获取Odom数据（每个数据包都更新姿态，确保实时性）
            robot_x = packet['odom_x']
            robot_y = packet['odom_y']
            robot_theta = packet['odom_theta']
            current_odom = (robot_x, robot_y, robot_theta)
            
            # 更新机器人位姿（如果没有使用独立Odom线程）
            if not USE_SEPARATE_ODOM_THREAD:
                self.lidar_data.robot_theta = robot_theta
                rx_grid = int(MAP_SIZE / 2 + (-robot_y) / MAP_RESOLUTION)
                ry_grid = int(MAP_SIZE / 2 + robot_x / MAP_RESOLUTION)
                self.lidar_data.robot_pos = (rx_grid, ry_grid)
            
            # 上位机额外过滤：进一步降低阈值以显示更多点
            if quality < 8 or distance > 1.5 or distance < 0.20:
                continue
            
            # 添加点、时间戳和机器人位姿（用于显示）
            current_time = time.time()
            self.lidar_data.angles.append(angle)
            self.lidar_data.distances.append(distance)
            self.lidar_data.qualities.append(quality)
            self.lidar_data.timestamps.append(current_time)
            self.lidar_data.robot_poses.append((robot_x, robot_y, robot_theta))  # 记录观测时的位姿
            
            # 🚀 添加到扫描缓冲区（用于批量插值校正）
            self.scan_buffer.add_point(angle, distance, quality, current_odom)
            
            # 收集待建图的点（延迟到第二阶段处理）
            pending_map_updates.append((angle, distance, robot_x, robot_y, robot_theta))
        
        # ========== 第二阶段：批量建图（低优先级，不阻塞位置更新） ==========
        
        # 🔧 提前检测旋转：如果当前正在旋转，跳过本帧所有建图
        is_rotating = self.slam.rotation_filter.is_rotation_detected()
        
        # 处理SYNC：批量插值校正建图
        if need_process_scan and not is_rotating:
            self.process_scan_buffer()  # 处理完整的上一圈（位姿插值，权重=8）
            if current_odom is not None:
                self.scan_buffer.start_new_scan(current_odom)  # 开始新的一圈
        
        # 实时建图：只在非旋转状态下处理本帧收集的点（权重=3）
        if not is_rotating:
            for angle, distance, robot_x, robot_y, robot_theta in pending_map_updates:
                # skip_rotation_check=True 因为外部已经检测过旋转状态
                self.slam.update_map(angle, distance, robot_x, robot_y, robot_theta, weight=3, skip_rotation_check=True)
        elif self.slam.DEBUG_MODE and len(pending_map_updates) > 0:
            print(f"🔄 Skipping {len(pending_map_updates)} points due to rotation (Status: {self.slam.rotation_filter.get_status()})")
        
        # 清理旧数据（根据配置选择模式）
        if USE_TIME_WINDOW:
            # 🕒 基于时间窗口清理（推荐：适合动态场景）
            if len(self.lidar_data.timestamps) > 100:  # 只在点数超过100时清理，避免频繁操作
                current_time = time.time()
                cutoff_time = current_time - MAX_DISPLAY_TIME
                
                # 找到第一个需要保留的索引（从后往前找更快）
                keep_from = 0
                for i in range(len(self.lidar_data.timestamps)):
                    if self.lidar_data.timestamps[i] >= cutoff_time:
                        keep_from = i
                        break
                
                # 批量删除过期数据
                if keep_from > 50:  # 只有积累足够多时才删除，减少操作频率
                    self.lidar_data.angles = self.lidar_data.angles[keep_from:]
                    self.lidar_data.distances = self.lidar_data.distances[keep_from:]
                    self.lidar_data.qualities = self.lidar_data.qualities[keep_from:]
                    self.lidar_data.timestamps = self.lidar_data.timestamps[keep_from:]
                    self.lidar_data.robot_poses = self.lidar_data.robot_poses[keep_from:]
        else:
            # 📊 基于固定点数清理（经典模式）
            if len(self.lidar_data.angles) > MAX_DISPLAY_POINTS:
                excess = len(self.lidar_data.angles) - MAX_DISPLAY_POINTS
                self.lidar_data.angles = self.lidar_data.angles[excess:]
                self.lidar_data.distances = self.lidar_data.distances[excess:]
                self.lidar_data.qualities = self.lidar_data.qualities[excess:]
                self.lidar_data.timestamps = self.lidar_data.timestamps[excess:]
                self.lidar_data.robot_poses = self.lidar_data.robot_poses[excess:]
        
        # 更新点云显示（应用角度镜像，与右侧SLAM地图保持一致）
        displayed_points = 0
        if len(self.lidar_data.angles) > 0:
            # 🚀 运动预测过滤：智能消除拖尾点
            if USE_MOTION_FILTER and current_odom and len(self.lidar_data.robot_poses) > 0:
                current_x, current_y, current_theta = current_odom
                valid_indices = []
                overlap_protected_indices = []  # 因重合保护而保留的索引
                
                # 第一步：基于运动距离过滤
                for i in range(len(self.lidar_data.angles)):
                    # 获取点被观测时的机器人位姿
                    obs_x, obs_y, obs_theta = self.lidar_data.robot_poses[i]
                    
                    # 计算机器人移动距离
                    dx = current_x - obs_x
                    dy = current_y - obs_y
                    motion_dist = np.sqrt(dx**2 + dy**2)
                    
                    # 如果机器人移动距离小于阈值，点仍然"跟得上"
                    if motion_dist < MOTION_THRESHOLD:
                        valid_indices.append(i)
                
                # 🧠 第二步：重合保护 - 检查被过滤掉的点是否和当前扫描重合
                if USE_OVERLAP_PROTECTION and len(valid_indices) < len(self.lidar_data.angles):
                    # 找出被过滤掉的点
                    filtered_out = set(range(len(self.lidar_data.angles))) - set(valid_indices)
                    
                    if len(filtered_out) > 0 and len(valid_indices) > 0:
                        # 计算当前帧点的世界坐标（最近的点，运动距离接近0）
                        current_frame_world = []
                        for i in valid_indices[-min(100, len(valid_indices)):]:  # 最近100个点作为当前帧
                            angle = self.lidar_data.angles[i]
                            dist = self.lidar_data.distances[i]
                            px, py, ptheta = self.lidar_data.robot_poses[i]
                            
                            # 转换到世界坐标系
                            angle_rad = np.deg2rad(angle)
                            world_x = px + dist * np.cos(angle_rad + np.deg2rad(ptheta))
                            world_y = py + dist * np.sin(angle_rad + np.deg2rad(ptheta))
                            current_frame_world.append((world_x, world_y))
                        
                        # 检查被过滤点是否与当前帧重合
                        overlap_count = 0
                        for i in filtered_out:
                            angle = self.lidar_data.angles[i]
                            dist = self.lidar_data.distances[i]
                            px, py, ptheta = self.lidar_data.robot_poses[i]
                            
                            # 历史点的世界坐标
                            angle_rad = np.deg2rad(angle)
                            hist_world_x = px + dist * np.cos(angle_rad + np.deg2rad(ptheta))
                            hist_world_y = py + dist * np.sin(angle_rad + np.deg2rad(ptheta))
                            
                            # 检查是否与当前帧任意点接近
                            for curr_x, curr_y in current_frame_world:
                                dist_to_current = np.sqrt((hist_world_x - curr_x)**2 + (hist_world_y - curr_y)**2)
                                if dist_to_current < OVERLAP_DISTANCE:
                                    overlap_count += 1
                                    overlap_protected_indices.append(i)
                                    break  # 找到一个重合就够了
                        
                        # 如果重合率高，保护这些点
                        overlap_ratio = overlap_count / len(filtered_out)
                        if overlap_ratio > OVERLAP_RATIO:
                            valid_indices.extend(overlap_protected_indices)
                            valid_indices.sort()  # 保持顺序
                
                # 过滤点
                if len(valid_indices) > 0:
                    angles_to_show = [self.lidar_data.angles[i] for i in valid_indices]
                    distances_to_show = [self.lidar_data.distances[i] for i in valid_indices]
                else:
                    angles_to_show = []
                    distances_to_show = []
                displayed_points = len(angles_to_show)
            else:
                # 不使用运动过滤，显示所有点
                angles_to_show = self.lidar_data.angles
                distances_to_show = self.lidar_data.distances
                displayed_points = len(angles_to_show)
            
            # 显示过滤后的点
            if len(angles_to_show) > 0:
                # RPLidar C1是顺时针，需要镜像为逆时针（标准右手系）
                mirrored_angles = [(360.0 - a) % 360.0 for a in angles_to_show]
                angles_rad = np.deg2rad(mirrored_angles)
                x_orig = np.array(distances_to_show) * np.cos(angles_rad)
                y_orig = np.array(distances_to_show) * np.sin(angles_rad)
                
                # 显示变换：逆时针旋转90° (x_display = -y, y_display = x)
                x_display = -y_orig
                y_display = x_orig
                self.scatter.set_offsets(np.c_[x_display, y_display])
            else:
                self.scatter.set_offsets(np.empty((0, 2)))
        
        # 更新地图显示
        self.map_img.set_data(self.lidar_data.grid_map)
        
        # 更新机器人轨迹显示
        if len(self.lidar_data.robot_trajectory) > 1:
            traj_array = np.array(self.lidar_data.robot_trajectory)
            self.trajectory_line.set_data(traj_array[:, 0], traj_array[:, 1])
        
        # 更新机器人当前位置和朝向（红色虚线射线）
        rx, ry = self.lidar_data.robot_pos
        robot_theta = self.lidar_data.robot_theta
        
        # 计算方向射线（从机器人位置延伸到地图边缘）
        # IMU: 0度=X正方向，逆时针为正
        ray_length = MAP_SIZE  # 射线长度延伸到地图边缘（栅格单位）
        ray_dx_orig = np.cos(np.deg2rad(robot_theta)) * ray_length  # 原始水平方向
        ray_dy_orig = np.sin(np.deg2rad(robot_theta)) * ray_length  # 原始垂直方向
        
        # 应用显示旋转变换：逆时针90° (x_display = -y, y_display = x)
        ray_dx = -ray_dy_orig
        ray_dy = ray_dx_orig
        
        # 更新方向射线（从机器人位置出发，延伸到地图边缘）
        self.robot_direction_line.set_data([rx, rx + ray_dx], [ry, ry + ray_dy])
        
        # 更新机器人中心点位置
        self.robot_center.set_data([rx], [ry])
        
        # Update status info (including Odom and heading)
        # 添加地图统计信息
        obstacle_cells = np.sum(self.lidar_data.grid_map > 0)
        max_obstacle_val = np.max(self.lidar_data.grid_map) if obstacle_cells > 0 else 0
        
        # 计算时间窗口跨度和过滤信息
        time_info = ""
        if USE_TIME_WINDOW and len(self.lidar_data.timestamps) > 1:
            time_span = self.lidar_data.timestamps[-1] - self.lidar_data.timestamps[0]
            time_info = f" ({time_span:.2f}s)"
        
        # 构建点数信息
        total_points = len(self.lidar_data.angles)
        if USE_MOTION_FILTER and displayed_points < total_points:
            points_info = f"{displayed_points}/{total_points}"
        else:
            points_info = f"{total_points}"
        
        if current_odom:
            # 🔄 统一坐标系：将IMU坐标转换为显示坐标系（与地图一致）
            # IMU坐标系 → 显示坐标系：x_display = -y_imu, y_display = x_imu
            imu_x, imu_y, imu_theta = current_odom
            display_x = -imu_y  # 显示X = -IMU_Y
            display_y = imu_x   # 显示Y = IMU_X
            
            # 横向展开显示（显示坐标系，与地图可视化一致）
            near_status = f"Near:{NEAR_CORRECTION_FACTOR:.2f}{'✓' if USE_NONLINEAR_CORRECTION else '✗'}"
            rotation_status = self.slam.rotation_filter.get_status()
            status = (f"Scans: {self.lidar_data.scan_count} | Points: {points_info}{time_info} | "
                     f"Pos: ({display_x:.2f}, {display_y:.2f})m | Heading: {imu_theta:.1f}° | "
                     f"Scale: {DISTANCE_SCALE_FACTOR:.2f}x | {near_status} | Rot:{rotation_status} | Obstacles: {obstacle_cells} | Max: {max_obstacle_val:.0f}")
            
            # 多线程模式：显示队列状态和性能统计
            if self.use_multithreading:
                queue_size = self.data_queue.qsize()
                recv_count = self.receiver_thread.packet_count
                odom_count = self.receiver_thread.odom_only_count
                lidar_count = self.receiver_thread.lidar_odom_count
                drop_count = self.receiver_thread.drop_count
                # 添加独立Odom包统计（🚀表示使用高速独立Odom通道）
                status += f" | Q:{queue_size} Odom:{odom_count} Lidar:{lidar_count} Drop:{drop_count}"
        else:
            near_status = f"Near:{NEAR_CORRECTION_FACTOR:.2f}{'✓' if USE_NONLINEAR_CORRECTION else '✗'}"
            rotation_status = self.slam.rotation_filter.get_status()
            status = (f"Scans: {self.lidar_data.scan_count} | Points: {points_info}{time_info} | "
                     f"Scale: {DISTANCE_SCALE_FACTOR:.2f}x | {near_status} | Rot:{rotation_status} | Obstacles: {obstacle_cells} | Max: {max_obstacle_val:.0f}")
        
        self.status_text.set_text(status)
        
        return self.scatter, self.map_img, self.trajectory_line, self.robot_direction_line, self.robot_center, self.status_text
    
    def send_precise_command(self, command):
        """发送精准控制指令到小车"""
        if not USE_BLUETOOTH_MODE:
            print(f"⚠️  精准控制仅在蓝牙模式下可用")
            return
        
        try:
            # 发送指令（需要加回车符）
            command_with_newline = command + '\n'
            self.serial.ser.write(command_with_newline.encode())
            print(f"📤 发送指令: {command}")
            
        except Exception as e:
            print(f"❌ 发送指令失败: {e}")
    
    def on_send_command(self, event):
        """发送按钮点击事件"""
        command = self.textbox.text.strip().upper()
        if command:
            self.send_precise_command(command)
            self.textbox.set_val('')  # 清空输入框
        else:
            print("⚠️  请输入指令")
    
    def on_textbox_submit(self, text):
        """文本框回车键提交事件"""
        command = text.strip().upper()
        if command:
            self.send_precise_command(command)
            self.textbox.set_val('')  # 清空输入框
    
    def on_key_press(self, event):
        """Keyboard press event handler"""
        # === Direction keys control robot (hold to execute) ===
        # Only enable robot control in Bluetooth mode
        if USE_BLUETOOTH_MODE:
            if event.key == 'up':
                # ↑: Forward
                if self.current_direction_key != 'up':
                    self.serial.ser.write(b'W')
                    self.current_direction_key = 'up'
                
            elif event.key == 'down':
                # ↓: Backward
                if self.current_direction_key != 'down':
                    self.serial.ser.write(b'S')
                    self.current_direction_key = 'down'
                
            elif event.key == 'left':
                # ←: Turn left
                if self.current_direction_key != 'left':
                    self.serial.ser.write(b'A')
                    self.current_direction_key = 'left'
                
            elif event.key == 'right':
                # →: Turn right
                if self.current_direction_key != 'right':
                    self.serial.ser.write(b'D')
                    self.current_direction_key = 'right'
                
            elif event.key == ' ':
                # Space: Stop
                self.serial.ser.write(b'x')
                self.current_direction_key = None
        
        # === 地图管理快捷键 ===
        elif event.key == 's':
            # S: Save map
            print("\n💾 Saving map...")
            metadata = {
                'scan_count': self.lidar_data.scan_count,
                'total_points': len(self.lidar_data.angles),
                'trajectory_length': len(self.lidar_data.robot_trajectory)
            }
            MapManager.save_map(
                self.lidar_data.grid_map,
                self.lidar_data.robot_trajectory,
                metadata=metadata
            )
            
        elif event.key == 'l':
            # L: List saved maps
            print("\n" + "="*50)
            saved_maps = MapManager.list_saved_maps()
            if saved_maps:
                print("\n💡 Tip: Use these files for map analysis")
            print("="*50)
            
        elif event.key == 'c':
            # C: Clear map
            print("\n🗑️  Clearing map...")
            self.lidar_data.grid_map = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.int8)
            self.lidar_data.robot_trajectory = []
            self.lidar_data.scan_count = 0
            print("✅ Map cleared")
            
        elif event.key == 'd':
            # D: Toggle debug mode
            self.slam.DEBUG_MODE = not self.slam.DEBUG_MODE
            status = "✅ ON" if self.slam.DEBUG_MODE else "❌ OFF"
            print(f"\n🔧 Coordinate transformation debug mode: {status}")
            if self.slam.DEBUG_MODE:
                print("   Will display: Lidar angle conversion, IMU angle, fused angle")
        
        # 🔧 距离缩放因子实时调整
        elif event.key == '=' or event.key == 'plus':
            # + (=) key: 增加距离缩放因子
            self.adjust_distance_scale(0.01)
            
        elif event.key == '-' or event.key == 'minus':
            # - key: 减少距离缩放因子
            self.adjust_distance_scale(-0.01)
            
        elif event.key == '0':
            # 0 key: 重置距离缩放因子
            self.reset_distance_scale()
        
        # 🔧 近距离非线性校正调整
        elif event.key == '[':
            # [ key: 减小近距离校正系数（向内收缩，修正外凸）
            self.adjust_near_correction(-0.02)
            
        elif event.key == ']':
            # ] key: 增加近距离校正系数（向外扩展）
            self.adjust_near_correction(0.02)
            
        elif event.key == 'n':
            # N key: 切换非线性校正开关
            self.toggle_nonlinear_correction()
            
        elif event.key == 'r':
            # R key: 切换旋转滤波开关
            self.toggle_rotation_filter()
            
        elif event.key == 'w':
            # W key: 切换墙保护开关
            self.toggle_wall_protection()
                
        elif event.key == 'q':
            # Q: Quit
            print("\n👋 Exiting program...")
            plt.close(self.fig)
    
    def adjust_distance_scale(self, delta):
        """调整距离缩放因子"""
        global DISTANCE_SCALE_FACTOR
        DISTANCE_SCALE_FACTOR = max(0.1, DISTANCE_SCALE_FACTOR + delta)  # 最小0.1避免负值
        action = "放大" if delta > 0 else "缩小"
        sign = "+" if delta > 0 else ""
        print(f"\n🔧 Distance Scale Factor: {DISTANCE_SCALE_FACTOR:.2f} ({sign}{delta:.2f}) - 雷达距离{action}")
    
    def reset_distance_scale(self):
        """重置距离缩放因子到默认值"""
        global DISTANCE_SCALE_FACTOR
        DISTANCE_SCALE_FACTOR = 1.0
        print(f"\n🔧 Distance Scale Factor: {DISTANCE_SCALE_FACTOR:.2f} (Reset) - 恢复默认比例")
    
    def adjust_near_correction(self, delta):
        """调整近距离校正系数"""
        global NEAR_CORRECTION_FACTOR
        NEAR_CORRECTION_FACTOR = max(0.5, min(1.5, NEAR_CORRECTION_FACTOR + delta))  # 限制在0.5-1.5之间
        action = "向外扩展" if delta > 0 else "向内收缩"
        sign = "+" if delta > 0 else ""
        print(f"\n🔧 Near Correction Factor: {NEAR_CORRECTION_FACTOR:.3f} ({sign}{delta:.02f}) - 近距离点{action}")
        print(f"   当前设置: <{NEAR_DISTANCE_THRESHOLD:.1f}m距离 × {NEAR_CORRECTION_FACTOR:.3f}")
    
    def toggle_nonlinear_correction(self):
        """切换非线性校正开关"""
        global USE_NONLINEAR_CORRECTION
        USE_NONLINEAR_CORRECTION = not USE_NONLINEAR_CORRECTION
        status = "✅ ON" if USE_NONLINEAR_CORRECTION else "❌ OFF"
        print(f"\n🔧 Nonlinear Distance Correction: {status}")
        if USE_NONLINEAR_CORRECTION:
            print(f"   近距离阈值: {NEAR_DISTANCE_THRESHOLD}m")
            print(f"   校正系数: {NEAR_CORRECTION_FACTOR:.3f}")
            print(f"   过渡范围: {CORRECTION_BLEND_RANGE}m")
    
    def toggle_rotation_filter(self):
        """切换旋转滤波开关"""
        global USE_ROTATION_FILTER
        USE_ROTATION_FILTER = not USE_ROTATION_FILTER
        status = "✅ ON" if USE_ROTATION_FILTER else "❌ OFF"
        print(f"\n🔧 Rotation Filter: {status}")
        if USE_ROTATION_FILTER:
            print(f"   角速度阈值: {ROTATION_THRESHOLD_DEG_S}°/s")
            print(f"   平滑窗口: {ROTATION_SMOOTH_WINDOW}个位姿")
            print(f"   恢复延迟: {ROTATION_FILTER_DELAY}s")
    
    def toggle_wall_protection(self):
        """切换墙保护开关"""
        global WALL_PROTECTION_ENABLED
        WALL_PROTECTION_ENABLED = not WALL_PROTECTION_ENABLED
        status = "✅ ON" if WALL_PROTECTION_ENABLED else "❌ OFF"
        print(f"\n🛡️  Wall Protection: {status}")
        if WALL_PROTECTION_ENABLED:
            print(f"   高置信度阈值: {HIGH_CONFIDENCE_THRESHOLD}")
            print(f"   高置信度侵蚀: {HIGH_CONFIDENCE_EROSION}/次")
            print(f"   普通侵蚀: {NORMAL_EROSION}/次")
    
    def on_key_release(self, event):
        """Keyboard release event handler"""
        # Brake immediately when direction key is released (only in Bluetooth mode)
        if USE_BLUETOOTH_MODE and event.key in ['up', 'down', 'left', 'right']:
            if self.current_direction_key == event.key:
                self.serial.ser.write(b'x')
                self.current_direction_key = None
    
    def start(self):
        """Start visualization"""
        ani = FuncAnimation(
            self.fig, 
            self.update, 
            interval=UPDATE_INTERVAL_MS,  # 更新间隔（可通过配置调整，越小越实时）
            blit=False
        )
        
        try:
            plt.show()
        finally:
            # 程序退出时清理线程
            if self.use_multithreading:
                print("\n🧹 Stopping receiver thread...")
                self.receiver_thread.stop()
                self.receiver_thread.join(timeout=1.0)  # 等待线程结束（最多1秒）
                print("✅ Receiver thread stopped")

# ==================== Main Function ====================
def main():
    print("=" * 50)
    print("  RPLidar C1 Visualizer")
    print("=" * 50)
    
    # Display mode
    mode_name = "Bluetooth" if USE_BLUETOOTH_MODE else "USB Serial (Debug)"
    print(f"\n📡 Mode: {mode_name}")
    print(f"📡 Connecting to: {SERIAL_PORT} @ {BAUD_RATE}")
    
    try:
        # Initialize serial receiver
        receiver = SerialReceiver(SERIAL_PORT, BAUD_RATE, mode=mode_name)
        
        # Start visualization
        print("\n🚀 Starting visualization...")
        print(f"\n⚡ Performance Settings:")
        print(f"  Update Rate: {1000/UPDATE_INTERVAL_MS:.0f} Hz ({UPDATE_INTERVAL_MS}ms interval)")
        print(f"  Processing: {POINTS_PER_FRAME} packets/frame")
        if USE_TIME_WINDOW:
            print(f"  Display Mode: Time Window ({MAX_DISPLAY_TIME*1000:.0f}ms) ⏱️")
        else:
            print(f"  Display Mode: Fixed Points ({MAX_DISPLAY_POINTS} pts) 📊")
        if USE_MOTION_FILTER:
            print(f"  🚀 Motion Filter: ENABLED (threshold={MOTION_THRESHOLD}m)")
            if USE_OVERLAP_PROTECTION:
                print(f"     🧠 Overlap Protection: ON (distance={OVERLAP_DISTANCE}m, ratio={OVERLAP_RATIO})")
        print(f"  Max Throughput: ~{POINTS_PER_FRAME * 1000/UPDATE_INTERVAL_MS:.0f} packets/sec")
        print(f"  Multi-threading: {'✅ ENABLED (High-performance CPU mode)' if USE_MULTITHREADING else '❌ Disabled'}")
        if USE_MULTITHREADING and USE_SEPARATE_ODOM_THREAD:
            print(f"\n🚀 独立Odom通道已启用:")
            print(f"  - IMU数据独立发送（250Hz高频更新）")
            print(f"  - 零延迟位姿更新（不受雷达传输速度限制）")
            print(f"  - 数据包更小（11字节 vs 18字节），传输更快")
            print(f"  - 实时性大幅提升：延迟从>100ms降至<5ms")
        print("\n💡 Control Tips:")
        print("  ↑ ↓ ← → : Forward/Backward/Left/Right (Hold to move, release to brake)")
        print("  Space   : Emergency stop")
        print("\n🎯 Precise Control (Right-bottom panel):")
        print("  Command Input: F[cm]=Forward, B[cm]=Backward, L[deg]=Left, R[deg]=Right")
        print("  Examples: F50 (forward 50cm), R90 (turn right 90°), L45 (turn left 45°)")
        print("  Quick Buttons: F10/F30/F50/F100, L90/L45/R45/R90")
        print("  Press Enter or click 'Send' to execute command")
        print("\n🗺️  Map Management:")
        print("  S       : Save map")
        print("  L       : List saved maps")
        print("  C       : Clear current map")
        print("  D       : Toggle coordinate debug mode")
        print("  W       : Toggle wall protection (prevent wall erosion)")
        print("  Q       : Quit program")
        print("\n🔧 Distance Calibration (左侧按钮):")
        print("  🖱️  GUI Buttons (Bottom Panel - LEFT): [+] [−] [0] - Click to adjust distance scale")
        print("     + Button : Increase distance scale (+0.01) - 雷达点离中心更远")
        print("     − Button : Decrease distance scale (-0.01) - 雷达点离中心更近") 
        print("     0 Button : Reset to default scale (1.0x) - 恢复默认比例")
        print("  📊 Status   : Current scale shown in top status bar (Scale: x.xxX)")
        print("  ⌨️  Keyboard : + / = (increase), - (decrease), 0 (reset) keys also work")
        print("\n🔧 Near-Distance Distortion Correction (右侧按钮 - 非线性畸变校正):")
        print("  🖱️  GUI Buttons (Bottom Panel - RIGHT): [◀] [▶] [N] - Click to adjust near correction")
        print("     ◀ Button : Decrease near correction (-0.02) - 近距离点向内收缩（修正外凸）")
        print("     ▶ Button : Increase near correction (+0.02) - 近距离点向外扩展")
        print("     N Button : Toggle nonlinear correction ON/OFF - 开关非线性校正")
        print(f"  💡 Current: {'✅ Enabled' if USE_NONLINEAR_CORRECTION else '❌ Disabled'} | "
              f"Factor={NEAR_CORRECTION_FACTOR:.3f} | Threshold={NEAR_DISTANCE_THRESHOLD}m")
        print("  ⌨️  Keyboard : [ (decrease), ] (increase), N (toggle) keys also work")
        print("\n📐 Coordinate System:")
        print("  Lidar: RPLidar C1 (Clockwise positive)")
        print("  IMU:   IM948 (Counter-clockwise positive)")
        print("  Auto conversion: ✅ Enabled")
        print("\n📐 坐标系说明:")
        print("  显示坐标系（统一）: X右→, Y上↑, θ逆时针为正")
        print("  IMU原始坐标系:     X前→, Y左↑, θ逆时针为正")
        print("  坐标变换:          X显示=-Y_IMU, Y显示=X_IMU (逆时针90°)")
        print("  Status显示:        显示坐标系（与地图一致）")
        
        if not USE_BLUETOOTH_MODE:
            print("\n⚠️  USB Debug Mode: Robot control disabled")
        
        print("")
        visualizer = LidarVisualizer(receiver, use_multithreading=USE_MULTITHREADING)
        visualizer.start()
        
    except serial.SerialException as e:
        print(f"❌ Serial error: {e}")
        print("\n💡 Please check:")
        if USE_BLUETOOTH_MODE:
            print("  1. Is Bluetooth module paired and connected?")
            print("  2. Is the serial port correct? (Windows: COMx, Linux: /dev/rfcomm0)")
            print(f"  3. Is the baud rate {BAUD_RATE}?")
        else:
            print("  1. Is the USB cable connected?")
            print(f"  2. Is the serial port correct? (Check Device Manager: {SERIAL_PORT})")
            print(f"  3. Is the baud rate {BAUD_RATE}?")
            print("  4. Is the Arduino IDE Serial Monitor CLOSED?")
    except KeyboardInterrupt:
        print("\n\n👋 Program exited")

if __name__ == "__main__":
    main()

