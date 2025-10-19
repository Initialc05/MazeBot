#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
迷宫可视化模块 - MazeBot导航系统

功能：
1. 迷宫地图可视化
2. 雷达点云显示
3. 置信度热力图
4. 实时状态监控

作者：MazeBot开发团队
日期：2025年1月14日
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import math
from typing import List, Tuple, Optional
from maze_mapper import MazeMap, WallSegment

class MazeVisualizer:
    """迷宫可视化器"""
    
    def __init__(self, figsize=(16, 8)):
        """
        初始化可视化器
        
        Args:
            figsize: 图形尺寸
        """
        self.figsize = figsize
        self.fig = None
        self.ax1 = None  # 网格地图
        self.ax2 = None  # 雷达点云
        self.ax3 = None  # 置信度热力图
        
        # 颜色配置
        self.colors = {
            'robot': 'blue',
            'trajectory': 'green',
            'walls': 'red',
            'lidar_points': 'lightblue',
            'entrance': 'green',
            'exit': 'red',
            'grid': 'lightgray'
        }
    
    def create_figure(self):
        """创建图形窗口"""
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=self.figsize)
        
        # 设置标题
        self.ax1.set_title('Grid-based Maze Map')
        self.ax2.set_title('Lidar Data and Wall Fitting')
        self.ax3.set_title('Horizontal Walls Confidence')
        self.ax4.set_title('Vertical Walls Confidence')
        
        # 设置坐标轴
        for ax in [self.ax1, self.ax2]:
            ax.set_xlim(-0.5, 4.5)
            ax.set_ylim(-0.5, 4.5)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
    
    def visualize_maze_map(self, maze_map: MazeMap, robot_pos: Optional[Tuple[float, float]] = None, 
                          robot_heading: Optional[float] = None, entrance_pos: Optional[Tuple[int, int]] = None,
                          exit_pos: Optional[Tuple[int, int]] = None):
        """
        可视化迷宫地图，包括墙壁、雷达数据点、机器人位置等
        
        Args:
            maze_map: 迷宫地图对象
            robot_pos: 机器人当前位置
            robot_heading: 机器人朝向
            entrance_pos: 入口位置
            exit_pos: 出口位置
        """
        if self.fig is None:
            self.create_figure()
        
        # 清空图形
        self.ax1.clear()
        self.ax2.clear()
        
        # 绘制网格地图
        self._draw_grid_map(maze_map, robot_pos, robot_heading, entrance_pos, exit_pos)
        
        # 绘制雷达点云和拟合结果
        self._draw_lidar_data(maze_map, robot_pos, robot_heading)
        
        # 绘制置信度热力图
        self._draw_confidence_heatmap(maze_map)
        
        # 设置标题和标签
        self.ax1.set_title('Grid-based Maze Map')
        self.ax2.set_title('Lidar Data and Wall Fitting')
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        self.ax2.set_xlabel('X')
        self.ax2.set_ylabel('Y')
        
        plt.tight_layout()
        plt.show()
    
    def _draw_grid_map(self, maze_map: MazeMap, robot_pos: Optional[Tuple[float, float]], 
                      robot_heading: Optional[float], entrance_pos: Optional[Tuple[int, int]],
                      exit_pos: Optional[Tuple[int, int]]):
        """绘制网格地图"""
        # 绘制网格线
        for i in range(6):
            self.ax1.axhline(i-0.5, color=self.colors['grid'], linewidth=0.5)
            self.ax1.axvline(i-0.5, color=self.colors['grid'], linewidth=0.5)
        
        # 绘制格点中心
        for x in range(5):
            for y in range(5):
                self.ax1.plot(x, y, 'ko', markersize=6)
        
        # 绘制墙壁
        self._draw_walls(maze_map, self.ax1)
        
        # 绘制入口和出口
        if entrance_pos:
            self.ax1.plot(entrance_pos[0], entrance_pos[1], 'go', markersize=12, label='Entrance')
        if exit_pos:
            self.ax1.plot(exit_pos[0], exit_pos[1], 'ro', markersize=12, label='Exit')
        
        # 绘制机器人位置
        if robot_pos:
            self.ax1.plot(robot_pos[0], robot_pos[1], 'bo', markersize=10, label='Robot')
            if robot_heading is not None:
                # 绘制朝向箭头
                dx = 0.3 * math.cos(math.radians(robot_heading))
                dy = 0.3 * math.sin(math.radians(robot_heading))
                self.ax1.arrow(robot_pos[0], robot_pos[1], dx, dy, 
                             head_width=0.1, head_length=0.1, fc='blue', ec='blue')
        
        # 绘制机器人轨迹
        if len(maze_map.robot_trajectory) > 1:
            traj_x = [pos[0] for pos in maze_map.robot_trajectory]
            traj_y = [pos[1] for pos in maze_map.robot_trajectory]
            self.ax1.plot(traj_x, traj_y, color=self.colors['trajectory'], linewidth=2, alpha=0.7, label='Trajectory')
        
        self.ax1.legend()
        self.ax1.set_xlim(-0.5, 4.5)
        self.ax1.set_ylim(-0.5, 4.5)
    
    def _draw_walls(self, maze_map: MazeMap, ax):
        """绘制墙壁"""
        # 绘制水平墙壁
        for x in range(4):
            for y in range(5):
                if maze_map.walls_h[x][y] is not None:
                    wall = maze_map.walls_h[x][y]
                    color_intensity = wall.confidence
                    ax.plot([x+0.5, x+0.5], [y-0.5, y+0.5], 
                           color=(color_intensity, 0, 0), linewidth=3, alpha=0.8)
        
        # 绘制垂直墙壁
        for x in range(5):
            for y in range(4):
                if maze_map.walls_v[x][y] is not None:
                    wall = maze_map.walls_v[x][y]
                    color_intensity = wall.confidence
                    ax.plot([x-0.5, x+0.5], [y+0.5, y+0.5], 
                           color=(color_intensity, 0, 0), linewidth=3, alpha=0.8)
    
    def _draw_lidar_data(self, maze_map: MazeMap, robot_pos: Optional[Tuple[float, float]], 
                        robot_heading: Optional[float]):
        """绘制雷达点云和拟合结果"""
        # 绘制雷达点云
        if maze_map.lidar_points:
            x_coords = [p[0] for p in maze_map.lidar_points]
            y_coords = [p[1] for p in maze_map.lidar_points]
            self.ax2.scatter(x_coords, y_coords, c=self.colors['lidar_points'], s=1, alpha=0.6, label='Lidar Points')
        
        # 绘制拟合的墙壁线段
        self._draw_walls(maze_map, self.ax2)
        
        # 绘制机器人位置
        if robot_pos:
            self.ax2.plot(robot_pos[0], robot_pos[1], 'bo', markersize=10, label='Robot')
            if robot_heading is not None:
                # 绘制朝向箭头
                dx = 0.3 * math.cos(math.radians(robot_heading))
                dy = 0.3 * math.sin(math.radians(robot_heading))
                self.ax2.arrow(robot_pos[0], robot_pos[1], dx, dy, 
                             head_width=0.1, head_length=0.1, fc='blue', ec='blue')
        
        self.ax2.legend()
        self.ax2.set_xlim(-0.5, 4.5)
        self.ax2.set_ylim(-0.5, 4.5)
    
    def _draw_confidence_heatmap(self, maze_map: MazeMap):
        """绘制置信度热力图"""
        # 水平墙壁置信度
        h_confidence = np.zeros((4, 5))
        for x in range(4):
            for y in range(5):
                if maze_map.walls_h[x][y] is not None:
                    h_confidence[x][y] = maze_map.walls_h[x][y].confidence
        
        im1 = self.ax3.imshow(h_confidence.T, cmap='Reds', aspect='equal', vmin=0, vmax=1)
        self.ax3.set_title('Horizontal Walls Confidence')
        self.ax3.set_xlabel('X')
        self.ax3.set_ylabel('Y')
        plt.colorbar(im1, ax=self.ax3)
        
        # 垂直墙壁置信度
        v_confidence = np.zeros((5, 4))
        for x in range(5):
            for y in range(4):
                if maze_map.walls_v[x][y] is not None:
                    v_confidence[x][y] = maze_map.walls_v[x][y].confidence
        
        im2 = self.ax4.imshow(v_confidence.T, cmap='Reds', aspect='equal', vmin=0, vmax=1)
        self.ax4.set_title('Vertical Walls Confidence')
        self.ax4.set_xlabel('X')
        self.ax4.set_ylabel('Y')
        plt.colorbar(im2, ax=self.ax4)
    
    def create_confidence_heatmap(self, maze_map: MazeMap):
        """创建独立的置信度热力图"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        # 水平墙壁置信度
        h_confidence = np.zeros((4, 5))
        for x in range(4):
            for y in range(5):
                if maze_map.walls_h[x][y] is not None:
                    h_confidence[x][y] = maze_map.walls_h[x][y].confidence
        
        im1 = ax1.imshow(h_confidence.T, cmap='Reds', aspect='equal', vmin=0, vmax=1)
        ax1.set_title('Horizontal Walls Confidence')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        plt.colorbar(im1, ax=ax1)
        
        # 垂直墙壁置信度
        v_confidence = np.zeros((5, 4))
        for x in range(5):
            for y in range(4):
                if maze_map.walls_v[x][y] is not None:
                    v_confidence[x][y] = maze_map.walls_v[x][y].confidence
        
        im2 = ax2.imshow(v_confidence.T, cmap='Reds', aspect='equal', vmin=0, vmax=1)
        ax2.set_title('Vertical Walls Confidence')
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        plt.colorbar(im2, ax=ax2)
        
        plt.tight_layout()
        plt.show()
    
    def save_maze_map(self, maze_map: MazeMap, filename: str):
        """
        保存迷宫地图到文件
        
        Args:
            maze_map: 迷宫地图对象
            filename: 文件名
        """
        self.visualize_maze_map(maze_map)
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"迷宫地图已保存到: {filename}")
    
    def create_animation(self, maze_map: MazeMap, robot_trajectory: List[Tuple[float, float]], 
                        robot_headings: List[float], interval: int = 100):
        """
        创建动画显示机器人运动轨迹
        
        Args:
            maze_map: 迷宫地图对象
            robot_trajectory: 机器人轨迹
            robot_headings: 机器人朝向历史
            interval: 动画间隔(毫秒)
        """
        if self.fig is None:
            self.create_figure()
        
        def animate(frame):
            if frame < len(robot_trajectory):
                robot_pos = robot_trajectory[frame]
                robot_heading = robot_headings[frame] if frame < len(robot_headings) else 0
                
                # 清空并重绘
                self.ax1.clear()
                self.ax2.clear()
                
                # 绘制到当前帧的轨迹
                current_trajectory = robot_trajectory[:frame+1]
                maze_map.robot_trajectory = current_trajectory
                
                self._draw_grid_map(maze_map, robot_pos, robot_heading, None, None)
                self._draw_lidar_data(maze_map, robot_pos, robot_heading)
        
        anim = FuncAnimation(self.fig, animate, frames=len(robot_trajectory), 
                           interval=interval, repeat=True)
        return anim
    
    def print_maze_statistics(self, maze_map: MazeMap):
        """打印迷宫统计信息"""
        print("\n=== 迷宫地图统计信息 ===")
        
        # 统计墙壁数量
        h_wall_count = sum(1 for x in range(4) for y in range(5) if maze_map.walls_h[x][y] is not None)
        v_wall_count = sum(1 for x in range(5) for y in range(4) if maze_map.walls_v[x][y] is not None)
        
        print(f"水平墙壁数量: {h_wall_count}")
        print(f"垂直墙壁数量: {v_wall_count}")
        print(f"总墙壁数量: {h_wall_count + v_wall_count}")
        
        # 统计置信度
        confidences = []
        for x in range(4):
            for y in range(5):
                if maze_map.walls_h[x][y] is not None:
                    confidences.append(maze_map.walls_h[x][y].confidence)
        
        for x in range(5):
            for y in range(4):
                if maze_map.walls_v[x][y] is not None:
                    confidences.append(maze_map.walls_v[x][y].confidence)
        
        if confidences:
            print(f"平均置信度: {np.mean(confidences):.3f}")
            print(f"最高置信度: {np.max(confidences):.3f}")
            print(f"最低置信度: {np.min(confidences):.3f}")
        
        # 统计雷达数据
        print(f"雷达点云数量: {len(maze_map.lidar_points)}")
        print(f"机器人轨迹点数: {len(maze_map.robot_trajectory)}")
        
        print("=" * 30)
