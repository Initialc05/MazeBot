#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
迷宫匹配快速测试脚本

功能：
1. 快速验证迷宫匹配功能
2. 使用模拟数据进行测试
3. 检查系统是否正常工作

作者：MazeBot开发团队
日期：2025年1月14日
"""

import numpy as np
import matplotlib.pyplot as plt
import time
import math
from typing import List, Tuple

from maze_mapper import MazeMapper
from maze_visualizer import MazeVisualizer
from maze_interface import MazeInterface

def generate_test_lidar_data(robot_pos: Tuple[float, float], robot_heading: float, 
                           walls: List[Tuple[float, float, float, float]], 
                           noise_level: float = 0.02) -> List[Tuple[float, float, int]]:
    """
    生成测试雷达数据
    
    Args:
        robot_pos: 机器人位置 (x, y)
        robot_heading: 机器人朝向角度
        walls: 墙壁列表 [(x1, y1, x2, y2), ...]
        noise_level: 噪声水平
    
    Returns:
        list: 雷达数据 [(angle, distance, quality), ...]
    """
    lidar_data = []
    
    # 生成360度扫描数据
    for angle in range(0, 360, 5):  # 每5度一个点
        min_distance = float('inf')
        
        # 计算射线与墙壁的交点
        for wall in walls:
            x1, y1, x2, y2 = wall
            
            # 射线参数
            ray_x = robot_pos[0] + math.cos(math.radians(angle + robot_heading))
            ray_y = robot_pos[1] + math.sin(math.radians(angle + robot_heading))
            
            # 计算交点
            intersection = line_intersection(
                robot_pos, (ray_x, ray_y),
                (x1, y1), (x2, y2)
            )
            
            if intersection:
                distance = math.sqrt((intersection[0] - robot_pos[0])**2 + 
                                   (intersection[1] - robot_pos[1])**2)
                if 0.1 < distance < 2.0:  # 有效距离范围
                    min_distance = min(min_distance, distance)
        
        # 如果没有检测到墙壁，使用最大距离
        if min_distance == float('inf'):
            min_distance = 2.0
        
        # 添加噪声
        distance = min_distance + np.random.normal(0, noise_level)
        distance = max(0.1, min(2.0, distance))  # 限制在有效范围内
        
        # 质量值（基于距离和噪声）
        quality = max(1, int(15 - distance * 5 - noise_level * 100))
        
        lidar_data.append((angle, distance, quality))
    
    return lidar_data

def line_intersection(p1: Tuple[float, float], p2: Tuple[float, float],
                     p3: Tuple[float, float], p4: Tuple[float, float]) -> Tuple[float, float]:
    """计算两条线的交点"""
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4
    
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-10:
        return None
    
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom
    
    if 0 <= t <= 1 and 0 <= u <= 1:
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        return (x, y)
    
    return None

def test_basic_functionality():
    """测试基本功能"""
    print("=== 测试基本功能 ===")
    
    # 创建迷宫匹配器
    entrance_pos = (0, 0)
    exit_pos = (4, 4)
    maze_mapper = MazeMapper(entrance_pos, exit_pos)
    
    # 定义测试墙壁
    test_walls = [
        (0.5, -0.5, 0.5, 0.5),   # 北边界
        (1.5, -0.5, 1.5, 0.5),   # 北边界
        (2.5, -0.5, 2.5, 0.5),   # 北边界
        (3.5, -0.5, 3.5, 0.5),   # 北边界
        (4.5, -0.5, 4.5, 0.5),   # 北边界
        (-0.5, 0.5, 0.5, 0.5),   # 西边界
        (-0.5, 1.5, 0.5, 1.5),   # 西边界
        (-0.5, 2.5, 0.5, 2.5),   # 西边界
        (-0.5, 3.5, 0.5, 3.5),   # 西边界
        (-0.5, 4.5, 0.5, 4.5),   # 西边界
    ]
    
    # 测试不同位置
    test_positions = [
        (0, 0, 0),    # 入口位置，朝北
        (1, 1, 90),   # 中心位置，朝东
        (2, 2, 180),  # 中心位置，朝南
        (3, 3, 270),  # 中心位置，朝西
    ]
    
    for x, y, heading in test_positions:
        print(f"\n测试位置: ({x}, {y}), 朝向: {heading}°")
        
        # 生成雷达数据
        lidar_data = generate_test_lidar_data((x, y), heading, test_walls)
        
        # 更新迷宫地图
        maze_mapper.update_maze_map(lidar_data, (x, y), heading)
        
        # 检查墙壁检测结果
        for direction in ['north', 'east', 'south', 'west']:
            has_wall = maze_mapper.is_wall_at(x, y, direction)
            confidence = maze_mapper.get_wall_confidence(x, y, direction)
            print(f"  {direction}: {'有墙壁' if has_wall else '无墙壁'}, 置信度: {confidence:.3f}")
    
    # 可视化结果
    visualizer = MazeVisualizer()
    maze_map = maze_mapper.get_wall_map()
    visualizer.visualize_maze_map(maze_map, (2, 2), 0, entrance_pos, exit_pos)
    
    print("基本功能测试完成")

def test_interface_integration():
    """测试接口集成"""
    print("\n=== 测试接口集成 ===")
    
    # 创建迷宫接口
    entrance_pos = (0, 0)
    exit_pos = (4, 4)
    
    config = {
        'visualization_enabled': True,
        'debug_mode': True,
        'update_interval': 0.05
    }
    
    maze_interface = MazeInterface(entrance_pos, exit_pos, config)
    
    # 启动接口
    maze_interface.start()
    
    try:
        # 模拟数据输入
        for i in range(20):
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
            
            if i % 5 == 0:
                maze_interface.print_status()
        
        # 可视化结果
        maze_interface.visualize_current_state()
        
    finally:
        maze_interface.stop()
    
    print("接口集成测试完成")

def test_performance():
    """测试性能"""
    print("\n=== 测试性能 ===")
    
    # 创建迷宫匹配器
    entrance_pos = (0, 0)
    exit_pos = (4, 4)
    maze_mapper = MazeMapper(entrance_pos, exit_pos)
    
    # 定义测试墙壁
    test_walls = [
        (0.5, -0.5, 0.5, 0.5),
        (1.5, -0.5, 1.5, 0.5),
        (2.5, -0.5, 2.5, 0.5),
        (3.5, -0.5, 3.5, 0.5),
        (4.5, -0.5, 4.5, 0.5),
    ]
    
    # 性能测试
    num_tests = 50
    total_time = 0
    
    for i in range(num_tests):
        # 随机位置和朝向
        x = np.random.uniform(0, 4)
        y = np.random.uniform(0, 4)
        heading = np.random.uniform(0, 360)
        
        # 生成雷达数据
        lidar_data = generate_test_lidar_data((x, y), heading, test_walls)
        
        # 测量更新时间
        start_time = time.time()
        maze_mapper.update_maze_map(lidar_data, (x, y), heading)
        end_time = time.time()
        
        total_time += (end_time - start_time)
    
    avg_time = total_time / num_tests
    print(f"平均更新时间: {avg_time*1000:.2f}ms")
    print(f"最大更新频率: {1/avg_time:.1f}Hz")
    
    # 内存使用测试
    maze_map = maze_mapper.get_wall_map()
    print(f"雷达点云数量: {len(maze_map.lidar_points)}")
    print(f"机器人轨迹点数: {len(maze_map.robot_trajectory)}")
    
    print("性能测试完成")

def main():
    """主测试函数"""
    print("迷宫匹配快速测试")
    print("=" * 50)
    
    try:
        # 运行所有测试
        test_basic_functionality()
        test_interface_integration()
        test_performance()
        
        print("\n所有测试完成！")
        print("如果测试通过，可以运行实机测试脚本：")
        print("python maze_test_integration.py")
        
    except Exception as e:
        print(f"测试过程中出现错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
