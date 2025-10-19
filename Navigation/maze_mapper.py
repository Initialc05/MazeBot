#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
迷宫匹配模块 - MazeBot导航系统

功能：
1. 基于雷达数据的墙壁检测和拟合
2. 4x4网格迷宫建模
3. 置信度评估和可视化
4. 与主系统的接口集成

作者：MazeBot开发团队
日期：2025年1月14日
"""

import numpy as np
import matplotlib.pyplot as plt
import math
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
import time

@dataclass
class WallSegment:
    """墙壁线段类"""
    start_pos: Tuple[float, float]  # (x, y) 起始位置
    end_pos: Tuple[float, float]    # (x, y) 结束位置
    confidence: float = 0.0         # 置信度 [0.0, 1.0]
    observations: List = None       # 观测历史
    length: float = 0.0             # 线段长度
    
    def __post_init__(self):
        if self.observations is None:
            self.observations = []
        self.length = self.calculate_length()
    
    def calculate_length(self) -> float:
        """计算线段长度"""
        dx = self.end_pos[0] - self.start_pos[0]
        dy = self.end_pos[1] - self.start_pos[1]
        return math.sqrt(dx*dx + dy*dy)
    
    def add_observation(self, quality: float):
        """添加观测记录"""
        self.observations.append({
            'quality': quality,
            'timestamp': time.time()
        })
        # 保持最近50个观测记录
        if len(self.observations) > 50:
            self.observations = self.observations[-50:]

class MazeMap:
    """迷宫地图类"""
    
    def __init__(self):
        # 水平墙壁：walls_h[x][y] 表示 (x+0.5, y) 位置的墙壁
        self.walls_h = [[None for _ in range(5)] for _ in range(4)]
        # 垂直墙壁：walls_v[x][y] 表示 (x, y+0.5) 位置的墙壁
        self.walls_v = [[None for _ in range(4)] for _ in range(5)]
        # 雷达点云数据
        self.lidar_points = []
        # 机器人轨迹
        self.robot_trajectory = []
        # 观测历史
        self.observation_history = []
    
    def clear_old_data(self, max_points=1000):
        """清理旧数据，保持性能"""
        if len(self.lidar_points) > max_points:
            self.lidar_points = self.lidar_points[-max_points:]
        if len(self.robot_trajectory) > 100:
            self.robot_trajectory = self.robot_trajectory[-100:]

class MazeMapper:
    """迷宫匹配主控制器"""
    
    def __init__(self, entrance_pos: Tuple[int, int], exit_pos: Tuple[int, int]):
        """
        初始化迷宫匹配器
        
        Args:
            entrance_pos: 入口位置 (x, y)
            exit_pos: 出口位置 (x, y)
        """
        self.maze_map = MazeMap()
        self.entrance_pos = entrance_pos
        self.exit_pos = exit_pos
        self.robot_pos = entrance_pos
        self.robot_heading = 0.0
        
        # 拟合参数
        self.min_points_for_fitting = 3
        self.confidence_threshold = 0.3
        self.direction_tolerance = 22.5  # 度
        
        # 初始化边界墙壁
        self.initialize_boundary_walls()
        
        print(f"迷宫匹配器初始化完成")
        print(f"入口位置: {entrance_pos}")
        print(f"出口位置: {exit_pos}")
    
    def initialize_boundary_walls(self):
        """初始化迷宫边界墙壁，除了出入口外全部设为墙壁"""
        print("初始化边界墙壁...")
        
        # 设置外围墙壁
        for x in range(5):
            for y in range(5):
                # 北边界 (y=0)
                if y == 0 and (x, y) != self.entrance_pos and (x, y) != self.exit_pos:
                    if x < 4:  # 水平墙壁
                        self.maze_map.walls_h[x][y] = WallSegment(
                            start_pos=(x+0.5, y-0.5),
                            end_pos=(x+0.5, y+0.5),
                            confidence=1.0
                        )
                
                # 南边界 (y=4)
                if y == 4 and (x, y) != self.entrance_pos and (x, y) != self.exit_pos:
                    if x < 4:  # 水平墙壁
                        self.maze_map.walls_h[x][y] = WallSegment(
                            start_pos=(x+0.5, y-0.5),
                            end_pos=(x+0.5, y+0.5),
                            confidence=1.0
                        )
                
                # 西边界 (x=0)
                if x == 0 and (x, y) != self.entrance_pos and (x, y) != self.exit_pos:
                    if y < 4:  # 垂直墙壁
                        self.maze_map.walls_v[x][y] = WallSegment(
                            start_pos=(x-0.5, y+0.5),
                            end_pos=(x+0.5, y+0.5),
                            confidence=1.0
                        )
                
                # 东边界 (x=4)
                if x == 4 and (x, y) != self.entrance_pos and (x, y) != self.exit_pos:
                    if y < 4:  # 垂直墙壁
                        self.maze_map.walls_v[x][y] = WallSegment(
                            start_pos=(x-0.5, y+0.5),
                            end_pos=(x+0.5, y+0.5),
                            confidence=1.0
                        )
        
        print("边界墙壁初始化完成")
    
    def update_maze_map(self, lidar_data: List[Tuple[float, float, int]], 
                       robot_pos: Tuple[float, float], robot_heading: float):
        """
        更新迷宫地图
        
        Args:
            lidar_data: 雷达点云数据 [(angle, distance, quality), ...]
            robot_pos: 机器人位置 (x, y)
            robot_heading: 机器人朝向角度
        """
        # 更新机器人状态
        self.robot_pos = robot_pos
        self.robot_heading = robot_heading
        
        # 保存雷达数据
        self.maze_map.lidar_points.extend(lidar_data)
        
        # 拟合墙壁
        fitted_walls = self.fit_walls_from_lidar(lidar_data, robot_pos, robot_heading)
        
        # 映射到网格
        self.map_walls_to_grid(fitted_walls)
        
        # 更新轨迹
        self.maze_map.robot_trajectory.append(robot_pos)
        
        # 清理旧数据
        self.maze_map.clear_old_data()
    
    def fit_walls_from_lidar(self, lidar_data: List[Tuple[float, float, int]], 
                           robot_pos: Tuple[float, float], robot_heading: float) -> List[WallSegment]:
        """
        从雷达数据拟合墙壁线段
        
        Args:
            lidar_data: 雷达点云数据 [(angle, distance, quality), ...]
            robot_pos: 机器人位置 (x, y)
            robot_heading: 机器人朝向角度
        
        Returns:
            list: 拟合出的墙壁线段列表
        """
        # 1. 将雷达点转换为世界坐标
        world_points = []
        for angle, distance, quality in lidar_data:
            if quality > 5 and 0.1 < distance < 1.5:  # 质量过滤
                world_x, world_y = self.polar_to_cartesian(angle, distance, robot_pos, robot_heading)
                world_points.append((world_x, world_y, quality))
        
        if len(world_points) < 3:
            return []
        
        # 2. 按方向聚类点云
        clustered_points = self.cluster_points_by_direction(world_points, robot_pos)
        
        # 3. 对每个聚类进行线段拟合
        fitted_walls = []
        for direction, points in clustered_points.items():
            if len(points) >= self.min_points_for_fitting:
                wall_segment = self.fit_line_segment(points, direction)
                if wall_segment and wall_segment.confidence > self.confidence_threshold:
                    fitted_walls.append(wall_segment)
        
        return fitted_walls
    
    def polar_to_cartesian(self, angle: float, distance: float, 
                          robot_pos: Tuple[float, float], robot_heading: float) -> Tuple[float, float]:
        """极坐标转笛卡尔坐标"""
        # 考虑机器人朝向
        absolute_angle = angle + robot_heading
        world_x = robot_pos[0] + distance * math.cos(math.radians(absolute_angle))
        world_y = robot_pos[1] + distance * math.sin(math.radians(absolute_angle))
        return world_x, world_y
    
    def cluster_points_by_direction(self, world_points: List[Tuple[float, float, int]], 
                                  robot_pos: Tuple[float, float]) -> Dict[str, List[Tuple[float, float, int]]]:
        """按方向聚类点云"""
        clusters = {
            'north': [],    # 0° ± 22.5°
            'east': [],     # 90° ± 22.5°
            'south': [],    # 180° ± 22.5°
            'west': []      # 270° ± 22.5°
        }
        
        for x, y, quality in world_points:
            # 计算相对于机器人的角度
            dx = x - robot_pos[0]
            dy = y - robot_pos[1]
            angle = math.degrees(math.atan2(dy, dx))
            
            # 归一化角度到 [0, 360)
            angle = (angle + 360) % 360
            
            # 分配到对应的方向聚类
            if 360 - self.direction_tolerance <= angle or angle < self.direction_tolerance:
                clusters['north'].append((x, y, quality))
            elif 90 - self.direction_tolerance <= angle < 90 + self.direction_tolerance:
                clusters['east'].append((x, y, quality))
            elif 180 - self.direction_tolerance <= angle < 180 + self.direction_tolerance:
                clusters['south'].append((x, y, quality))
            elif 270 - self.direction_tolerance <= angle < 270 + self.direction_tolerance:
                clusters['west'].append((x, y, quality))
        
        return clusters
    
    def fit_line_segment(self, points: List[Tuple[float, float, int]], direction: str) -> Optional[WallSegment]:
        """
        使用最小二乘法拟合直线段
        
        Args:
            points: 点云数据 [(x, y, quality), ...]
            direction: 方向 ('north', 'east', 'south', 'west')
        
        Returns:
            WallSegment: 拟合的墙壁线段
        """
        if len(points) < self.min_points_for_fitting:
            return None
        
        # 提取x, y坐标
        x_coords = [p[0] for p in points]
        y_coords = [p[1] for p in points]
        qualities = [p[2] for p in points]
        
        # 根据方向选择拟合方法
        if direction in ['north', 'south']:
            # 水平墙壁：拟合 y = constant
            y_mean = np.average(y_coords, weights=qualities)
            confidence = self.calculate_confidence(points, 'horizontal', y_mean)
            
            # 确定线段端点
            x_min, x_max = min(x_coords), max(x_coords)
            start_pos = (x_min, y_mean)
            end_pos = (x_max, y_mean)
            
        else:  # east, west
            # 垂直墙壁：拟合 x = constant
            x_mean = np.average(x_coords, weights=qualities)
            confidence = self.calculate_confidence(points, 'vertical', x_mean)
            
            # 确定线段端点
            y_min, y_max = min(y_coords), max(y_coords)
            start_pos = (x_mean, y_min)
            end_pos = (x_mean, y_max)
        
        wall_segment = WallSegment(start_pos, end_pos, confidence)
        
        # 添加观测记录
        for _, _, quality in points:
            wall_segment.add_observation(quality)
        
        return wall_segment
    
    def calculate_confidence(self, points: List[Tuple[float, float, int]], 
                           wall_type: str, fitted_value: float) -> float:
        """
        计算拟合置信度
        
        Args:
            points: 点云数据
            wall_type: 墙壁类型 ('horizontal' 或 'vertical')
            fitted_value: 拟合值
        
        Returns:
            float: 置信度 [0.0, 1.0]
        """
        if len(points) < 3:
            return 0.0
        
        # 计算点到拟合线的距离
        distances = []
        for x, y, quality in points:
            if wall_type == 'horizontal':
                distance = abs(y - fitted_value)
            else:  # vertical
                distance = abs(x - fitted_value)
            distances.append(distance)
        
        # 计算标准差
        mean_distance = np.mean(distances)
        std_distance = np.std(distances)
        
        # 置信度计算：基于点数和拟合质量
        point_confidence = min(1.0, len(points) / 10.0)  # 点数贡献
        quality_confidence = max(0.0, 1.0 - std_distance / 0.1)  # 拟合质量贡献
        
        return (point_confidence + quality_confidence) / 2.0
    
    def map_walls_to_grid(self, fitted_walls: List[WallSegment]):
        """
        将拟合的墙壁线段映射到网格墙壁
        
        Args:
            fitted_walls: 拟合的墙壁线段列表
        """
        for wall_segment in fitted_walls:
            # 判断是水平墙壁还是垂直墙壁
            if abs(wall_segment.start_pos[1] - wall_segment.end_pos[1]) < 0.1:
                # 水平墙壁
                y = wall_segment.start_pos[1]
                x_start = wall_segment.start_pos[0]
                x_end = wall_segment.end_pos[0]
                
                # 映射到网格
                grid_y = round(y)
                if 0 <= grid_y < 5:
                    for x in np.arange(x_start, x_end, 0.1):
                        grid_x = round(x - 0.5)
                        if 0 <= grid_x < 4:
                            if (self.maze_map.walls_h[grid_x][grid_y] is None or 
                                self.maze_map.walls_h[grid_x][grid_y].confidence < wall_segment.confidence):
                                self.maze_map.walls_h[grid_x][grid_y] = wall_segment
            
            else:
                # 垂直墙壁
                x = wall_segment.start_pos[0]
                y_start = wall_segment.start_pos[1]
                y_end = wall_segment.end_pos[1]
                
                # 映射到网格
                grid_x = round(x)
                if 0 <= grid_x < 5:
                    for y in np.arange(y_start, y_end, 0.1):
                        grid_y = round(y - 0.5)
                        if 0 <= grid_y < 4:
                            if (self.maze_map.walls_v[grid_x][grid_y] is None or 
                                self.maze_map.walls_v[grid_x][grid_y].confidence < wall_segment.confidence):
                                self.maze_map.walls_v[grid_x][grid_y] = wall_segment
    
    def get_wall_map(self) -> MazeMap:
        """获取墙壁地图"""
        return self.maze_map
    
    def get_robot_pos(self) -> Tuple[float, float]:
        """获取机器人当前位置"""
        return self.robot_pos
    
    def get_robot_heading(self) -> float:
        """获取机器人当前朝向"""
        return self.robot_heading
    
    def is_wall_at(self, x: float, y: float, direction: str) -> bool:
        """
        检查指定位置是否有墙壁
        
        Args:
            x, y: 位置坐标
            direction: 方向 ('north', 'east', 'south', 'west')
        
        Returns:
            bool: 是否有墙壁
        """
        grid_x, grid_y = round(x), round(y)
        
        if direction == 'north':
            if 0 <= grid_x < 4 and 0 <= grid_y < 5:
                return self.maze_map.walls_h[grid_x][grid_y] is not None
        elif direction == 'south':
            if 0 <= grid_x < 4 and 0 <= grid_y < 5:
                return self.maze_map.walls_h[grid_x][grid_y] is not None
        elif direction == 'east':
            if 0 <= grid_x < 5 and 0 <= grid_y < 4:
                return self.maze_map.walls_v[grid_x][grid_y] is not None
        elif direction == 'west':
            if 0 <= grid_x < 5 and 0 <= grid_y < 4:
                return self.maze_map.walls_v[grid_x][grid_y] is not None
        
        return False
    
    def get_wall_confidence(self, x: float, y: float, direction: str) -> float:
        """
        获取指定位置墙壁的置信度
        
        Args:
            x, y: 位置坐标
            direction: 方向
        
        Returns:
            float: 置信度 [0.0, 1.0]
        """
        grid_x, grid_y = round(x), round(y)
        
        if direction == 'north':
            if 0 <= grid_x < 4 and 0 <= grid_y < 5:
                wall = self.maze_map.walls_h[grid_x][grid_y]
                return wall.confidence if wall else 0.0
        elif direction == 'south':
            if 0 <= grid_x < 4 and 0 <= grid_y < 5:
                wall = self.maze_map.walls_h[grid_x][grid_y]
                return wall.confidence if wall else 0.0
        elif direction == 'east':
            if 0 <= grid_x < 5 and 0 <= grid_y < 4:
                wall = self.maze_map.walls_v[grid_x][grid_y]
                return wall.confidence if wall else 0.0
        elif direction == 'west':
            if 0 <= grid_x < 5 and 0 <= grid_y < 4:
                wall = self.maze_map.walls_v[grid_x][grid_y]
                return wall.confidence if wall else 0.0
        
        return 0.0
