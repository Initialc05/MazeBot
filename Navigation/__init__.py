#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MazeBot导航系统包

功能：
1. 迷宫匹配和墙壁检测
2. 自动寻路算法
3. 可视化系统
4. 与主系统接口

作者：MazeBot开发团队
日期：2025年1月14日
"""

from .maze_mapper import MazeMapper, MazeMap, WallSegment
from .maze_visualizer import MazeVisualizer
from .maze_interface import MazeInterface, create_maze_interface

__version__ = "1.0.0"
__author__ = "MazeBot开发团队"

__all__ = [
    'MazeMapper',
    'MazeMap', 
    'WallSegment',
    'MazeVisualizer',
    'MazeInterface',
    'create_maze_interface'
]
