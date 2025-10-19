#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
迷宫接口模块 - MazeBot导航系统

功能：
1. 与主系统的接口集成
2. 数据格式转换
3. 实时状态监控
4. 配置管理

作者：MazeBot开发团队
日期：2025年1月14日
"""

import time
import threading
from typing import List, Tuple, Optional, Dict, Any
from queue import Queue
import numpy as np

from maze_mapper import MazeMapper, MazeMap
from maze_visualizer import MazeVisualizer

class MazeInterface:
    """迷宫接口类 - 与主系统集成"""
    
    def __init__(self, entrance_pos: Tuple[int, int], exit_pos: Tuple[int, int], 
                 config: Optional[Dict[str, Any]] = None):
        """
        初始化迷宫接口
        
        Args:
            entrance_pos: 入口位置 (x, y)
            exit_pos: 出口位置 (x, y)
            config: 配置参数
        """
        # 默认配置
        self.config = {
            'min_points_for_fitting': 3,
            'confidence_threshold': 0.3,
            'direction_tolerance': 22.5,
            'max_lidar_points': 1000,
            'max_trajectory_points': 100,
            'update_interval': 0.1,  # 秒
            'visualization_enabled': True,
            'debug_mode': False
        }
        
        # 更新配置
        if config:
            self.config.update(config)
        
        # 初始化组件
        self.maze_mapper = MazeMapper(entrance_pos, exit_pos)
        self.visualizer = MazeVisualizer() if self.config['visualization_enabled'] else None
        
        # 数据队列
        self.lidar_queue = Queue()
        self.odom_queue = Queue()
        
        # 状态变量
        self.is_running = False
        self.last_update_time = 0
        self.update_thread = None
        
        # 统计信息
        self.stats = {
            'total_updates': 0,
            'walls_detected': 0,
            'avg_confidence': 0.0,
            'last_update_duration': 0.0
        }
        
        print(f"迷宫接口初始化完成")
        print(f"配置参数: {self.config}")
    
    def start(self):
        """启动迷宫接口"""
        if self.is_running:
            print("迷宫接口已在运行")
            return
        
        self.is_running = True
        self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.update_thread.start()
        
        print("迷宫接口已启动")
    
    def stop(self):
        """停止迷宫接口"""
        self.is_running = False
        if self.update_thread:
            self.update_thread.join(timeout=1.0)
        
        print("迷宫接口已停止")
    
    def _update_loop(self):
        """更新循环"""
        while self.is_running:
            try:
                current_time = time.time()
                
                # 检查是否需要更新
                if current_time - self.last_update_time >= self.config['update_interval']:
                    self._process_queues()
                    self.last_update_time = current_time
                
                time.sleep(0.01)  # 避免CPU占用过高
                
            except Exception as e:
                print(f"更新循环错误: {e}")
                time.sleep(0.1)
    
    def _process_queues(self):
        """处理数据队列"""
        start_time = time.time()
        
        # 处理雷达数据
        lidar_batch = []
        while not self.lidar_queue.empty():
            try:
                lidar_data = self.lidar_queue.get_nowait()
                lidar_batch.extend(lidar_data)
            except:
                break
        
        # 处理里程计数据
        robot_pos = None
        robot_heading = None
        while not self.odom_queue.empty():
            try:
                odom_data = self.odom_queue.get_nowait()
                robot_pos = (odom_data['x'], odom_data['y'])
                robot_heading = odom_data['heading']
            except:
                break
        
        # 更新迷宫地图
        if lidar_batch and robot_pos is not None:
            self.maze_mapper.update_maze_map(lidar_batch, robot_pos, robot_heading)
            self.stats['total_updates'] += 1
            
            # 更新统计信息
            self._update_statistics()
            
            # 可视化更新
            if self.visualizer and self.config['visualization_enabled']:
                self._update_visualization()
        
        # 记录更新时间
        self.stats['last_update_duration'] = time.time() - start_time
    
    def _update_statistics(self):
        """更新统计信息"""
        maze_map = self.maze_mapper.get_wall_map()
        
        # 统计墙壁数量
        wall_count = 0
        confidences = []
        
        for x in range(4):
            for y in range(5):
                if maze_map.walls_h[x][y] is not None:
                    wall_count += 1
                    confidences.append(maze_map.walls_h[x][y].confidence)
        
        for x in range(5):
            for y in range(4):
                if maze_map.walls_v[x][y] is not None:
                    wall_count += 1
                    confidences.append(maze_map.walls_v[x][y].confidence)
        
        self.stats['walls_detected'] = wall_count
        self.stats['avg_confidence'] = np.mean(confidences) if confidences else 0.0
    
    def _update_visualization(self):
        """更新可视化"""
        try:
            maze_map = self.maze_mapper.get_wall_map()
            robot_pos = self.maze_mapper.get_robot_pos()
            robot_heading = self.maze_mapper.get_robot_heading()
            
            # 这里可以实现实时可视化更新
            # 为了避免阻塞，可以设置一个标志来控制更新频率
            pass
            
        except Exception as e:
            if self.config['debug_mode']:
                print(f"可视化更新错误: {e}")
    
    def add_lidar_data(self, lidar_data: List[Tuple[float, float, int]]):
        """
        添加雷达数据
        
        Args:
            lidar_data: 雷达数据 [(angle, distance, quality), ...]
        """
        if not self.is_running:
            return
        
        try:
            self.lidar_queue.put_nowait(lidar_data)
        except:
            # 队列满时丢弃旧数据
            try:
                self.lidar_queue.get_nowait()
                self.lidar_queue.put_nowait(lidar_data)
            except:
                pass
    
    def add_odom_data(self, x: float, y: float, heading: float):
        """
        添加里程计数据
        
        Args:
            x, y: 位置坐标
            heading: 朝向角度
        """
        if not self.is_running:
            return
        
        odom_data = {
            'x': x,
            'y': y,
            'heading': heading,
            'timestamp': time.time()
        }
        
        try:
            self.odom_queue.put_nowait(odom_data)
        except:
            # 队列满时丢弃旧数据
            try:
                self.odom_queue.get_nowait()
                self.odom_queue.put_nowait(odom_data)
            except:
                pass
    
    def get_wall_map(self) -> MazeMap:
        """获取墙壁地图"""
        return self.maze_mapper.get_wall_map()
    
    def get_robot_state(self) -> Tuple[float, float, float]:
        """获取机器人状态"""
        robot_pos = self.maze_mapper.get_robot_pos()
        robot_heading = self.maze_mapper.get_robot_heading()
        return robot_pos[0], robot_pos[1], robot_heading
    
    def is_wall_at(self, x: float, y: float, direction: str) -> bool:
        """检查指定位置是否有墙壁"""
        return self.maze_mapper.is_wall_at(x, y, direction)
    
    def get_wall_confidence(self, x: float, y: float, direction: str) -> float:
        """获取指定位置墙壁的置信度"""
        return self.maze_mapper.get_wall_confidence(x, y, direction)
    
    def get_statistics(self) -> Dict[str, Any]:
        """获取统计信息"""
        return self.stats.copy()
    
    def visualize_current_state(self):
        """可视化当前状态"""
        if not self.visualizer:
            print("可视化功能未启用")
            return
        
        maze_map = self.maze_mapper.get_wall_map()
        robot_pos = self.maze_mapper.get_robot_pos()
        robot_heading = self.maze_mapper.get_robot_heading()
        
        self.visualizer.visualize_maze_map(
            maze_map, robot_pos, robot_heading, 
            self.maze_mapper.entrance_pos, self.maze_mapper.exit_pos
        )
    
    def save_maze_map(self, filename: str):
        """保存迷宫地图"""
        if not self.visualizer:
            print("可视化功能未启用")
            return
        
        maze_map = self.maze_mapper.get_wall_map()
        self.visualizer.save_maze_map(maze_map, filename)
    
    def print_status(self):
        """打印状态信息"""
        print("\n=== 迷宫接口状态 ===")
        print(f"运行状态: {'运行中' if self.is_running else '已停止'}")
        print(f"总更新次数: {self.stats['total_updates']}")
        print(f"检测到的墙壁数量: {self.stats['walls_detected']}")
        print(f"平均置信度: {self.stats['avg_confidence']:.3f}")
        print(f"上次更新耗时: {self.stats['last_update_duration']*1000:.1f}ms")
        
        # 队列状态
        print(f"雷达数据队列大小: {self.lidar_queue.qsize()}")
        print(f"里程计数据队列大小: {self.odom_queue.qsize()}")
        
        # 机器人状态
        robot_pos = self.maze_mapper.get_robot_pos()
        robot_heading = self.maze_mapper.get_robot_heading()
        print(f"机器人位置: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})")
        print(f"机器人朝向: {robot_heading:.1f}°")
        
        print("=" * 25)
    
    def set_config(self, key: str, value: Any):
        """设置配置参数"""
        if key in self.config:
            self.config[key] = value
            print(f"配置已更新: {key} = {value}")
        else:
            print(f"未知配置项: {key}")
    
    def get_config(self, key: str) -> Any:
        """获取配置参数"""
        return self.config.get(key, None)

# 工厂函数
def create_maze_interface(entrance_pos: Tuple[int, int], exit_pos: Tuple[int, int], 
                         config: Optional[Dict[str, Any]] = None) -> MazeInterface:
    """
    创建迷宫接口实例
    
    Args:
        entrance_pos: 入口位置
        exit_pos: 出口位置
        config: 配置参数
    
    Returns:
        MazeInterface: 迷宫接口实例
    """
    return MazeInterface(entrance_pos, exit_pos, config)

# 示例使用
if __name__ == "__main__":
    # 创建迷宫接口
    entrance = (0, 0)
    exit_pos = (4, 4)
    
    config = {
        'visualization_enabled': True,
        'debug_mode': True,
        'update_interval': 0.05
    }
    
    maze_interface = create_maze_interface(entrance, exit_pos, config)
    
    # 启动接口
    maze_interface.start()
    
    try:
        # 模拟数据输入
        for i in range(100):
            # 模拟雷达数据
            lidar_data = [
                (0, 0.5, 10),    # 前方0.5米
                (90, 0.3, 8),    # 右侧0.3米
                (180, 0.7, 12),  # 后方0.7米
                (270, 0.4, 9)    # 左侧0.4米
            ]
            maze_interface.add_lidar_data(lidar_data)
            
            # 模拟里程计数据
            x = i * 0.1
            y = 0.0
            heading = 0.0
            maze_interface.add_odom_data(x, y, heading)
            
            time.sleep(0.1)
            
            if i % 10 == 0:
                maze_interface.print_status()
    
    except KeyboardInterrupt:
        print("用户中断")
    
    finally:
        # 停止接口
        maze_interface.stop()
        print("程序结束")
