#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
迷宫匹配实机测试集成脚本

功能：
1. 集成到现有的lidar_visualizer系统
2. 实时接收雷达和里程计数据
3. 运行迷宫匹配算法
4. 实时可视化迷宫地图

作者：MazeBot开发团队
日期：2025年1月14日
"""

import sys
import os
import time
import threading
import queue
from typing import List, Tuple, Optional

# 添加主系统路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'HOST'))

# 导入主系统模块
import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import matplotlib.patches as patches

# 导入迷宫匹配模块
from maze_mapper import MazeMapper, MazeMap
from maze_visualizer import MazeVisualizer
from maze_interface import MazeInterface

# ==================== 配置参数 ====================
# 串口配置（与主系统保持一致）
USE_BLUETOOTH_MODE = True
if USE_BLUETOOTH_MODE:
    SERIAL_PORT = 'COM18'  # 蓝牙串口
    BAUD_RATE = 921600     # 蓝牙波特率
else:
    SERIAL_PORT = 'COM7'   # USB串口
    BAUD_RATE = 115200     # USB串口波特率

# 迷宫配置
ENTRANCE_POS = (0, 0)  # 入口位置
EXIT_POS = (4, 4)      # 出口位置

# 迷宫匹配配置
MAZE_CONFIG = {
    'min_points_for_fitting': 3,
    'confidence_threshold': 0.3,
    'direction_tolerance': 22.5,
    'max_lidar_points': 1000,
    'max_trajectory_points': 100,
    'update_interval': 0.1,
    'visualization_enabled': True,
    'debug_mode': True
}

# ==================== 串口数据接收 ====================
class SerialReceiver:
    """串口数据接收器（简化版）"""
    
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        print(f"串口已连接: {port}, 波特率: {baudrate}")
        time.sleep(1)
    
    def read_packet(self):
        """读取一个完整的数据包"""
        # 寻找包头 0x55 0xAA (0xAA55的little-endian存储)
        while True:
            byte1 = self.ser.read(1)
            if len(byte1) == 0:
                return None
                
            if byte1[0] == 0x55:
                byte2 = self.ser.read(1)
                if len(byte2) > 0 and byte2[0] == 0xAA:
                    break
        
        # 读取剩余16字节数据
        data = self.ser.read(16)
        if len(data) < 16:
            return None
        
        # 解析数据包
        angle_q8, distance_mm, quality, odom_x_cm, odom_y_cm, odom_theta_q8, checksum = struct.unpack('<IHBhhiB', data)
        
        # 校验
        calculated_checksum = 0
        for byte in data[:-1]:  # 除了最后的checksum
            calculated_checksum ^= byte
        
        if calculated_checksum != checksum:
            return None
        
        # 转换数据格式
        angle_deg = angle_q8 / 256.0
        distance_m = distance_mm / 1000.0
        odom_x = odom_x_cm / 100.0
        odom_y = odom_y_cm / 100.0
        odom_theta = odom_theta_q8 / 256.0
        
        return {
            'type': 'LIDAR_ODOM',
            'angle': angle_deg,
            'distance': distance_m,
            'quality': quality,
            'odom_x': odom_x,
            'odom_y': odom_y,
            'odom_theta': odom_theta
        }
    
    def close(self):
        """关闭串口"""
        if self.ser.is_open:
            self.ser.close()

# ==================== 迷宫测试可视化器 ====================
class MazeTestVisualizer:
    """迷宫测试可视化器"""
    
    def __init__(self, maze_interface):
        self.maze_interface = maze_interface
        self.fig = None
        self.ax1 = None  # 原始雷达数据
        self.ax2 = None  # 迷宫地图
        self.ax3 = None  # 置信度热力图
        
        # 统计信息
        self.packet_count = 0
        self.last_update_time = time.time()
        
        # 创建图形界面
        self.create_figure()
    
    def create_figure(self):
        """创建图形窗口"""
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        
        # 设置标题
        self.ax1.set_title('原始雷达数据')
        self.ax2.set_title('迷宫地图')
        self.ax3.set_title('水平墙壁置信度')
        self.ax4.set_title('垂直墙壁置信度')
        
        # 设置坐标轴
        self.ax1.set_xlim(-2, 2)
        self.ax1.set_ylim(-2, 2)
        self.ax1.set_aspect('equal')
        self.ax1.grid(True, alpha=0.3)
        
        self.ax2.set_xlim(-0.5, 4.5)
        self.ax2.set_ylim(-0.5, 4.5)
        self.ax2.set_aspect('equal')
        self.ax2.grid(True, alpha=0.3)
        
        # 添加控制按钮
        ax_button = plt.axes([0.02, 0.02, 0.1, 0.05])
        self.button_save = Button(ax_button, '保存地图')
        self.button_save.on_clicked(self.save_maze_map)
        
        plt.tight_layout()
    
    def update_display(self, lidar_data, robot_pos, robot_heading):
        """更新显示"""
        # 清空图形
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        self.ax4.clear()
        
        # 绘制原始雷达数据
        self.draw_lidar_data(lidar_data, robot_pos, robot_heading)
        
        # 绘制迷宫地图
        self.draw_maze_map(robot_pos, robot_heading)
        
        # 绘制置信度热力图
        self.draw_confidence_heatmap()
        
        # 更新统计信息
        self.update_statistics()
        
        # 刷新显示
        plt.draw()
        plt.pause(0.01)
    
    def draw_lidar_data(self, lidar_data, robot_pos, robot_heading):
        """绘制原始雷达数据"""
        self.ax1.set_title(f'原始雷达数据 (包数: {self.packet_count})')
        self.ax1.set_xlim(-2, 2)
        self.ax1.set_ylim(-2, 2)
        self.ax1.set_aspect('equal')
        self.ax1.grid(True, alpha=0.3)
        
        if lidar_data:
            # 转换雷达数据到世界坐标
            world_points = []
            for angle, distance, quality in lidar_data:
                if quality > 5 and 0.1 < distance < 1.5:
                    world_x = robot_pos[0] + distance * np.cos(np.radians(angle + robot_heading))
                    world_y = robot_pos[1] + distance * np.sin(np.radians(angle + robot_heading))
                    world_points.append((world_x, world_y))
            
            if world_points:
                x_coords = [p[0] for p in world_points]
                y_coords = [p[1] for p in world_points]
                self.ax1.scatter(x_coords, y_coords, c='lightblue', s=1, alpha=0.6)
        
        # 绘制机器人位置
        self.ax1.plot(robot_pos[0], robot_pos[1], 'bo', markersize=8, label='Robot')
        if robot_heading is not None:
            dx = 0.3 * np.cos(np.radians(robot_heading))
            dy = 0.3 * np.sin(np.radians(robot_heading))
            self.ax1.arrow(robot_pos[0], robot_pos[1], dx, dy, 
                         head_width=0.1, head_length=0.1, fc='blue', ec='blue')
        
        self.ax1.legend()
    
    def draw_maze_map(self, robot_pos, robot_heading):
        """绘制迷宫地图"""
        self.ax2.set_title('迷宫地图')
        self.ax2.set_xlim(-0.5, 4.5)
        self.ax2.set_ylim(-0.5, 4.5)
        self.ax2.set_aspect('equal')
        self.ax2.grid(True, alpha=0.3)
        
        # 绘制网格
        for i in range(6):
            self.ax2.axhline(i-0.5, color='lightgray', linewidth=0.5)
            self.ax2.axvline(i-0.5, color='lightgray', linewidth=0.5)
        
        # 绘制格点中心
        for x in range(5):
            for y in range(5):
                self.ax2.plot(x, y, 'ko', markersize=4)
        
        # 绘制入口和出口
        self.ax2.plot(ENTRANCE_POS[0], ENTRANCE_POS[1], 'go', markersize=10, label='入口')
        self.ax2.plot(EXIT_POS[0], EXIT_POS[1], 'ro', markersize=10, label='出口')
        
        # 获取迷宫地图
        maze_map = self.maze_interface.get_wall_map()
        
        # 绘制墙壁
        for x in range(4):
            for y in range(5):
                if maze_map.walls_h[x][y] is not None:
                    wall = maze_map.walls_h[x][y]
                    color_intensity = wall.confidence
                    self.ax2.plot([x+0.5, x+0.5], [y-0.5, y+0.5], 
                               color=(color_intensity, 0, 0), linewidth=3, alpha=0.8)
        
        for x in range(5):
            for y in range(4):
                if maze_map.walls_v[x][y] is not None:
                    wall = maze_map.walls_v[x][y]
                    color_intensity = wall.confidence
                    self.ax2.plot([x-0.5, x+0.5], [y+0.5, y+0.5], 
                               color=(color_intensity, 0, 0), linewidth=3, alpha=0.8)
        
        # 绘制机器人位置
        self.ax2.plot(robot_pos[0], robot_pos[1], 'bo', markersize=8, label='机器人')
        if robot_heading is not None:
            dx = 0.3 * np.cos(np.radians(robot_heading))
            dy = 0.3 * np.sin(np.radians(robot_heading))
            self.ax2.arrow(robot_pos[0], robot_pos[1], dx, dy, 
                         head_width=0.1, head_length=0.1, fc='blue', ec='blue')
        
        # 绘制机器人轨迹
        if len(maze_map.robot_trajectory) > 1:
            traj_x = [pos[0] for pos in maze_map.robot_trajectory]
            traj_y = [pos[1] for pos in maze_map.robot_trajectory]
            self.ax2.plot(traj_x, traj_y, 'g-', linewidth=2, alpha=0.7, label='轨迹')
        
        self.ax2.legend()
    
    def draw_confidence_heatmap(self):
        """绘制置信度热力图"""
        maze_map = self.maze_interface.get_wall_map()
        
        # 水平墙壁置信度
        h_confidence = np.zeros((4, 5))
        for x in range(4):
            for y in range(5):
                if maze_map.walls_h[x][y] is not None:
                    h_confidence[x][y] = maze_map.walls_h[x][y].confidence
        
        im1 = self.ax3.imshow(h_confidence.T, cmap='Reds', aspect='equal', vmin=0, vmax=1)
        self.ax3.set_title('水平墙壁置信度')
        self.ax3.set_xlabel('X')
        self.ax3.set_ylabel('Y')
        
        # 垂直墙壁置信度
        v_confidence = np.zeros((5, 4))
        for x in range(5):
            for y in range(4):
                if maze_map.walls_v[x][y] is not None:
                    v_confidence[x][y] = maze_map.walls_v[x][y].confidence
        
        im2 = self.ax4.imshow(v_confidence.T, cmap='Reds', aspect='equal', vmin=0, vmax=1)
        self.ax4.set_title('垂直墙壁置信度')
        self.ax4.set_xlabel('X')
        self.ax4.set_ylabel('Y')
    
    def update_statistics(self):
        """更新统计信息"""
        current_time = time.time()
        if current_time - self.last_update_time >= 1.0:  # 每秒更新一次
            stats = self.maze_interface.get_statistics()
            print(f"\n=== 迷宫匹配统计 ===")
            print(f"总更新次数: {stats['total_updates']}")
            print(f"检测到的墙壁数量: {stats['walls_detected']}")
            print(f"平均置信度: {stats['avg_confidence']:.3f}")
            print(f"上次更新耗时: {stats['last_update_duration']*1000:.1f}ms")
            print(f"数据包数量: {self.packet_count}")
            print("=" * 25)
            
            self.last_update_time = current_time
    
    def save_maze_map(self, event):
        """保存迷宫地图"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"maze_map_{timestamp}.png"
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"迷宫地图已保存到: {filename}")
    
    def increment_packet_count(self):
        """增加数据包计数"""
        self.packet_count += 1

# ==================== 主测试类 ====================
class MazeTestRunner:
    """迷宫测试运行器"""
    
    def __init__(self):
        self.serial_receiver = None
        self.maze_interface = None
        self.visualizer = None
        self.running = False
        
        # 数据缓存
        self.lidar_buffer = []
        self.last_robot_pos = (0, 0)
        self.last_robot_heading = 0.0
        
    def initialize(self):
        """初始化系统"""
        print("=== 迷宫匹配实机测试初始化 ===")
        
        try:
            # 初始化串口接收器
            self.serial_receiver = SerialReceiver(SERIAL_PORT, BAUD_RATE)
            print("✅ 串口连接成功")
            
            # 初始化迷宫接口
            self.maze_interface = MazeInterface(ENTRANCE_POS, EXIT_POS, MAZE_CONFIG)
            self.maze_interface.start()
            print("✅ 迷宫接口启动成功")
            
            # 初始化可视化器
            self.visualizer = MazeTestVisualizer(self.maze_interface)
            print("✅ 可视化器初始化成功")
            
            print("=== 初始化完成 ===")
            return True
            
        except Exception as e:
            print(f"❌ 初始化失败: {e}")
            return False
    
    def run(self):
        """运行测试"""
        if not self.initialize():
            return
        
        print("\n=== 开始迷宫匹配测试 ===")
        print("按 Ctrl+C 停止测试")
        
        self.running = True
        
        try:
            while self.running:
                # 读取数据包
                packet = self.serial_receiver.read_packet()
                
                if packet is None:
                    continue
                
                if packet == 'SYNC':
                    # 处理完整的雷达扫描
                    if self.lidar_buffer:
                        self.process_lidar_scan()
                        self.lidar_buffer.clear()
                    continue
                
                if not isinstance(packet, dict) or packet.get('type') != 'LIDAR_ODOM':
                    continue
                
                # 提取数据
                angle = packet['angle']
                distance = packet['distance']
                quality = packet['quality']
                robot_x = packet['odom_x']
                robot_y = packet['odom_y']
                robot_theta = packet['odom_theta']
                
                # 更新机器人状态
                self.last_robot_pos = (robot_x, robot_y)
                self.last_robot_heading = robot_theta
                
                # 缓存雷达数据
                self.lidar_buffer.append((angle, distance, quality))
                
                # 添加数据到迷宫接口
                self.maze_interface.add_lidar_data([(angle, distance, quality)])
                self.maze_interface.add_odom_data(robot_x, robot_y, robot_theta)
                
                # 更新显示
                self.visualizer.update_display(
                    self.lidar_buffer, 
                    self.last_robot_pos, 
                    self.last_robot_heading
                )
                
                # 增加数据包计数
                self.visualizer.increment_packet_count()
                
                # 控制更新频率
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n用户中断测试")
        except Exception as e:
            print(f"测试过程中出现错误: {e}")
        finally:
            self.cleanup()
    
    def process_lidar_scan(self):
        """处理完整的雷达扫描"""
        if not self.lidar_buffer:
            return
        
        print(f"处理雷达扫描: {len(self.lidar_buffer)} 个点")
        
        # 这里可以添加额外的扫描处理逻辑
        # 例如：扫描质量评估、异常检测等
    
    def cleanup(self):
        """清理资源"""
        print("\n=== 清理资源 ===")
        
        self.running = False
        
        if self.maze_interface:
            self.maze_interface.stop()
            print("✅ 迷宫接口已停止")
        
        if self.serial_receiver:
            self.serial_receiver.close()
            print("✅ 串口连接已关闭")
        
        if self.visualizer and self.visualizer.fig:
            plt.close(self.visualizer.fig)
            print("✅ 图形界面已关闭")
        
        print("=== 清理完成 ===")

# ==================== 主函数 ====================
def main():
    """主函数"""
    print("迷宫匹配实机测试程序")
    print("=" * 50)
    
    # 检查串口配置
    print(f"串口配置:")
    print(f"  模式: {'蓝牙' if USE_BLUETOOTH_MODE else 'USB'}")
    print(f"  端口: {SERIAL_PORT}")
    print(f"  波特率: {BAUD_RATE}")
    print(f"迷宫配置:")
    print(f"  入口位置: {ENTRANCE_POS}")
    print(f"  出口位置: {EXIT_POS}")
    print("=" * 50)
    
    # 创建并运行测试
    test_runner = MazeTestRunner()
    test_runner.run()

if __name__ == "__main__":
    main()
