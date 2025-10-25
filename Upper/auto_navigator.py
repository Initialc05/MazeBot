#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动导航系统 - 基于墙跟随算法
支持沿右墙/左墙保持固定距离行进，从起点到终点再返回
"""

import argparse
import math
import queue
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple, Set
import heapq

import numpy as np

# 导入SLAM相关模块
from lidar_visualizer import (
    SerialReceiver,
    DataReceiverThread,
    SharedOdomData,
    SimpleGridSLAM,
    LidarData,
    MapManager,
    MAP_SIZE,
    MAP_RESOLUTION,
    MAX_RANGE,
)


# ========== 数据结构 ==========

@dataclass
class RobotState:
    """机器人状态"""
    x: float
    y: float
    theta_deg: float
    linear_speed: float = 0.0
    angular_speed: float = 0.0


@dataclass
class GridCell:
    """栅格单元"""
    x: int
    y: int
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y


@dataclass
class AStarNode:
    """A*算法节点"""
    cell: GridCell
    g_cost: float
    h_cost: float
    f_cost: float
    parent: Optional['AStarNode'] = None
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost


# ========== 工具函数 ==========

def world_to_grid(x: float, y: float) -> GridCell:
    """世界坐标转栅格坐标"""
    grid_x = int(MAP_SIZE // 2 + x / MAP_RESOLUTION)
    grid_y = int(MAP_SIZE // 2 + y / MAP_RESOLUTION)
    return GridCell(grid_x, grid_y)


def grid_to_world(cell: GridCell) -> Tuple[float, float]:
    """栅格坐标转世界坐标"""
    x = (cell.x - MAP_SIZE // 2) * MAP_RESOLUTION
    y = (cell.y - MAP_SIZE // 2) * MAP_RESOLUTION
    return (x, y)


def normalize_angle(angle_deg: float) -> float:
    """角度归一化到[-180, 180]"""
    while angle_deg > 180:
        angle_deg -= 360
    while angle_deg < -180:
        angle_deg += 360
    return angle_deg


def angle_diff(target_deg: float, current_deg: float) -> float:
    """计算角度差（最短路径）"""
    diff = target_deg - current_deg
    return normalize_angle(diff)


def get_neighbors(cell: GridCell) -> List[GridCell]:
    """获取8邻域邻居"""
    neighbors = []
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0:
                continue
            new_cell = GridCell(cell.x + dx, cell.y + dy)
            if 0 <= new_cell.x < MAP_SIZE and 0 <= new_cell.y < MAP_SIZE:
                neighbors.append(new_cell)
    return neighbors


def is_obstacle(grid_map: np.ndarray, cell: GridCell) -> bool:
    """检查栅格是否为障碍物"""
    if not (0 <= cell.x < MAP_SIZE and 0 <= cell.y < MAP_SIZE):
        return True
    return grid_map[cell.y, cell.x] > 0


def calculate_distance(cell1: GridCell, cell2: GridCell) -> float:
    """计算两个栅格之间的距离"""
    dx = cell1.x - cell2.x
    dy = cell1.y - cell2.y
    return math.sqrt(dx*dx + dy*dy)


# ========== A*路径规划 ==========

class AStarPlanner:
    """纯粹的A*路径规划算法"""
    
    def __init__(self, grid_map: np.ndarray):
        self.grid_map = grid_map
    
    def plan(self, start_world: Tuple[float, float], goal_world: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        使用A*算法规划路径
        
        Args:
            start_world: 起点世界坐标 (x, y)
            goal_world: 终点世界坐标 (x, y)
        
        Returns:
            路径点列表（世界坐标）
        """
        start_cell = world_to_grid(start_world[0], start_world[1])
        goal_cell = world_to_grid(goal_world[0], goal_world[1])
        
        # 检查起点和终点是否有效
        if is_obstacle(self.grid_map, start_cell):
            print(f"警告: 起点 {start_world} 在障碍物上")
            return []
        
        if is_obstacle(self.grid_map, goal_cell):
            print(f"警告: 终点 {goal_world} 在障碍物上")
            return []

        # A*算法
        open_set = []
        closed_set: Set[GridCell] = set()
        came_from = {}
        g_score = {start_cell: 0}
        f_score = {start_cell: self._heuristic(start_cell, goal_cell)}

        heapq.heappush(open_set, (f_score[start_cell], start_cell))

        while open_set:
            current_f, current_cell = heapq.heappop(open_set)
            
            if current_cell == goal_cell:
                # 重构路径
                path_cells = self._reconstruct_path(came_from, current_cell)
                return [grid_to_world(cell) for cell in path_cells]
            
            closed_set.add(current_cell)
            
            for neighbor in get_neighbors(current_cell):
                if neighbor in closed_set or is_obstacle(self.grid_map, neighbor):
                    continue
                
                tentative_g = g_score[current_cell] + calculate_distance(current_cell, neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current_cell
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal_cell)
                    
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        print("A*算法未找到路径")
        return []

    def _heuristic(self, cell1: GridCell, cell2: GridCell) -> float:
        """启发式函数（欧几里得距离）"""
        return calculate_distance(cell1, cell2)
    
    def _reconstruct_path(self, came_from: dict, current: GridCell) -> List[GridCell]:
        """重构路径"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


# ========== DWA局部避障 ==========

class DWALocalPlanner:
    """DWA局部避障规划器，支持90度和180度转向"""
    
    def __init__(self):
        # 运动参数
        self.max_linear_speed = 0.3  # m/s
        self.max_angular_speed = 90.0  # deg/s
        self.min_linear_speed = 0.0
        self.min_angular_speed = 0.0
        
        # 预测参数
        self.predict_time = 1.0  # 预测时间
        self.dt = 0.1  # 时间步长
        
        # 评价函数权重
        self.alpha = 1.0  # 朝向权重
        self.beta = 1.0   # 速度权重
        self.gamma = 1.0  # 障碍物权重
        
        # 安全距离
        self.safety_distance = 0.2  # 10cm安全距离
        
        # 支持的动作：直线前进、90度左转、90度右转、180度转向
        self.actions = [
            ('W', 0.0, 0.0),      # 直行
            ('A', 0.0, 90.0),     # 左转90度
            ('D', 0.0, -90.0),    # 右转90度
            ('A', 0.0, 180.0),    # 左转180度
            ('D', 0.0, -180.0),   # 右转180度
            ('x', 0.0, 0.0),      # 停止
        ]
    
    def plan(self, 
             current_state: RobotState, 
               goal_world: Tuple[float, float],
             obstacles_world: List[Tuple[float, float]]) -> Tuple[str, float]:
        """
        规划下一步动作
        
        Args:
            current_state: 当前机器人状态
            goal_world: 目标点世界坐标
            obstacles_world: 障碍物点列表
        
        Returns:
            (命令, 执行时间)
        """
        best_action = 'x'
        best_score = -float('inf')
        best_duration = 0.1
        
        for action, linear_vel, angular_vel in self.actions:
            # 模拟执行动作
            score = self._evaluate_action(
                current_state, action, linear_vel, angular_vel, 
                goal_world, obstacles_world
            )
            
            if score > best_score:
                best_score = score
                best_action = action
                # 转向动作需要更长时间
                if abs(angular_vel) > 0:
                    best_duration = abs(angular_vel) / self.max_angular_speed
                else:
                    best_duration = 0.1
        
        return best_action, best_duration
    
    def _evaluate_action(self,
                            state: RobotState,
                         action: str,
                         linear_vel: float,
                         angular_vel: float,
                            goal_world: Tuple[float, float],
                            obstacles_world: List[Tuple[float, float]]) -> float:
        """评价动作的得分"""
        
        # 模拟执行动作后的状态
        predicted_state = self._simulate_motion(state, linear_vel, angular_vel)
        
        # 朝向得分（朝向目标越好得分越高）
        goal_angle = math.degrees(math.atan2(
            goal_world[1] - predicted_state.y,
            goal_world[0] - predicted_state.x
        ))
        angle_diff = abs(angle_diff(goal_angle, predicted_state.theta_deg))
        heading_score = max(0, 1.0 - angle_diff / 180.0)
        
        # 速度得分（适当的速度得分更高）
        velocity_score = min(1.0, linear_vel / self.max_linear_speed)
        
        # 障碍物得分（离障碍物越远得分越高）
        obstacle_score = self._calculate_obstacle_score(predicted_state, obstacles_world)
        
        # 综合得分
        total_score = (self.alpha * heading_score + 
                      self.beta * velocity_score + 
                      self.gamma * obstacle_score)
        
        return total_score
    
    def _simulate_motion(self, 
                        state: RobotState, 
                        linear_vel: float, 
                        angular_vel: float) -> RobotState:
        """模拟运动"""
        dt = self.dt
        steps = int(self.predict_time / dt)
        
        x, y, theta = state.x, state.y, state.theta_deg
        
        for _ in range(steps):
            # 更新位置
            x += linear_vel * math.cos(math.radians(theta)) * dt
            y += linear_vel * math.sin(math.radians(theta)) * dt
            theta += angular_vel * dt
            theta = normalize_angle(theta)
        
        return RobotState(x, y, theta, linear_vel, angular_vel)
    
    def _calculate_obstacle_score(self, 
                                 state: RobotState, 
                                 obstacles_world: List[Tuple[float, float]]) -> float:
        """计算障碍物得分"""
        if not obstacles_world:
            return 1.0
        
        min_distance = float('inf')
        for obs_x, obs_y in obstacles_world:
            distance = math.sqrt((state.x - obs_x)**2 + (state.y - obs_y)**2)
            min_distance = min(min_distance, distance)
        
        if min_distance >= self.safety_distance:
            return 1.0
        elif min_distance <= 0.1:  # 太近了
            return 0.0
        else:
            return min_distance / self.safety_distance


# ========== 墙跟随控制器 ==========

class WallFollower:
    """墙跟随控制器，支持右墙跟随和左墙跟随"""
    
    def __init__(self, target_distance: float = 0.35):
        """
        初始化墙跟随控制器
        
        Args:
            target_distance: 目标墙距（米），默认35cm
        """
        self.target_distance = target_distance
        
        # PID参数（调整为更稳定的值）
        self.kp = 1.5  # 比例系数
        self.ki = 0.0  # 积分系数（暂时不使用，避免积分饱和）
        self.kd = 0.3  # 微分系数
        
        # PID状态
        self.prev_error = 0.0
        self.integral = 0.0
        
        # 距离容差（考虑传感器误差）
        self.distance_tolerance = 0.08  # 8cm容差，考虑实际传感器误差
        
        # 前方障碍物检测距离
        self.forward_obstacle_threshold = 0.35  # 35cm，给转向留出空间
        
        # 命令平滑
        self.command_history = []
        self.max_history_length = 3
        
    def reset_pid(self):
        """重置PID状态"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.command_history = []
    
    def get_wall_distance_and_angle(self, 
                                    lidar_points: List[Tuple[float, float, float]], 
                                    side: str = 'right') -> Tuple[Optional[float], Optional[float]]:
        """
        从激光雷达数据中提取墙壁距离和角度
        
        Args:
            lidar_points: [(angle, distance, timestamp), ...] 激光雷达点
            side: 'right' 或 'left'，检测右侧或左侧墙壁
        
        Returns:
            (wall_distance, wall_angle) 墙壁距离和相对角度，如果检测不到返回(None, None)
        """
        if not lidar_points:
            return None, None
        
        # 定义侧面扫描角度范围
        # 激光雷达坐标系：0度=前，90度=左，180度=后，270度=右
        if side == 'right':
            # 右侧：240-300度范围（右后到右前）
            angle_range = (240, 300)
        else:  # left
            # 左侧：60-120度范围（左前到左后）
            angle_range = (60, 120)
        
        # 收集侧面的距离测量
        side_distances = []
        for angle, distance, _ in lidar_points:
            # 将角度归一化到0-360
            angle_norm = angle % 360
            
            if angle_range[0] <= angle_norm <= angle_range[1]:
                if 0.15 < distance < 1.5:  # 过滤无效距离
                    side_distances.append((angle_norm, distance))
        
        if not side_distances:
            return None, None
        
        # 使用中位数和平均值结合，更稳定地估计墙距
        side_distances.sort(key=lambda x: x[1])
        
        # 如果点数太少，使用所有点
        if len(side_distances) <= 3:
            avg_distance = sum(d for _, d in side_distances) / len(side_distances)
            avg_angle = sum(a for a, _ in side_distances) / len(side_distances)
        else:
            # 使用中间的60%点（去除最近和最远的20%）
            start_idx = len(side_distances) // 5
            end_idx = len(side_distances) * 4 // 5
            middle_points = side_distances[start_idx:end_idx] if end_idx > start_idx else side_distances
            
            if not middle_points:
                middle_points = side_distances
            
            avg_distance = sum(d for _, d in middle_points) / len(middle_points)
            avg_angle = sum(a for a, _ in middle_points) / len(middle_points)
        
        return avg_distance, avg_angle
    
    def check_forward_obstacle(self, lidar_points: List[Tuple[float, float, float]]) -> bool:
        """
        检查前方是否有障碍物
        
        Args:
            lidar_points: [(angle, distance, timestamp), ...]
        
        Returns:
            True表示前方有障碍物
        """
        if not lidar_points:
            return False
        
        # 前方角度范围：0度附近（±30度）
        forward_angles = [(330, 360), (0, 30)]
        
        for angle, distance, _ in lidar_points:
            angle_norm = angle % 360
            
            for start, end in forward_angles:
                if start <= angle_norm <= end or (start > end and (angle_norm >= start or angle_norm <= end)):
                    if distance < self.forward_obstacle_threshold:
                        return True
        
        return False
    
    def calculate_control(self, 
                         current_distance: float, 
                         side: str = 'right',
                         dt: float = 0.1) -> str:
        """
        根据当前墙距计算控制命令
        
        Args:
            current_distance: 当前与墙的距离（米）
            side: 'right' 或 'left'
            dt: 时间间隔
        
        Returns:
            控制命令：'W' (直行), 'A' (左转), 'D' (右转)
        """
        # 计算误差
        error = current_distance - self.target_distance
        
        # PID计算
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        
        control = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        self.prev_error = error
        
        # 根据侧面和误差决定转向
        if side == 'right':
            # 右墙跟随：如果距离太近（error < 0），需要左转（'A'）
            # 如果距离太远（error > 0），需要右转（'D'）
            if error < -self.distance_tolerance:
                cmd = 'A'  # 离墙太近，左转
            elif error > self.distance_tolerance:
                cmd = 'D'  # 离墙太远，右转
            else:
                cmd = 'W'  # 距离合适，直行
        else:  # left
            # 左墙跟随：如果距离太近（error < 0），需要右转（'D'）
            # 如果距离太远（error > 0），需要左转（'A'）
            if error < -self.distance_tolerance:
                cmd = 'D'  # 离墙太近，右转
            elif error > self.distance_tolerance:
                cmd = 'A'  # 离墙太远，左转
            else:
                cmd = 'W'  # 距离合适，直行
        
        # 命令平滑：记录历史命令
        self.command_history.append(cmd)
        if len(self.command_history) > self.max_history_length:
            self.command_history.pop(0)
        
        # 如果最近3次命令中直行占多数，优先直行
        if len(self.command_history) >= 2:
            w_count = self.command_history.count('W')
            if w_count >= len(self.command_history) // 2 and 'W' in self.command_history:
                return 'W'
        
        return cmd


# ========== 主控制器 ==========

class AutoNavigator:
    """自动导航控制器"""
    
    def __init__(self,
                 port: str,
                 baud: int = 115200,
                 start_world: Optional[Tuple[float, float]] = None,
                 goal_world: Optional[Tuple[float, float]] = None,
                 wall_distance: float = 0.35):
        self.port = port
        self.baud = baud
        # 固定起点和终点 - 280cm x 280cm地图
        self.start_world = start_world if start_world else (0.0, 0.0)
        self.goal_world = goal_world if goal_world else (1.05, 2.45)  # 105cm, 245cm转换为米
        self.wall_distance = wall_distance  # 目标墙距35cm
        
        # 通信
        self.serial = SerialReceiver(port, baud, mode="AutoNav")
        self.data_queue = queue.Queue(maxsize=20000)
        self.shared_odom = SharedOdomData()
        self.receiver_thread = DataReceiverThread(
            self.serial, self.data_queue, shared_odom=self.shared_odom
        )
        self.receiver_thread.daemon = True

        # SLAM
        self.lidar_data = LidarData()
        self.slam = SimpleGridSLAM(self.lidar_data)

        # 墙跟随控制器
        self.wall_follower = WallFollower(target_distance=wall_distance)

        # 控制状态
        self.last_command = None
        self.returning = False
        self.at_goal = False
        self.navigation_phase = 'going'  # 'going' 或 'returning'
        
        # 激光雷达数据缓存
        self.lidar_points_cache = []  # [(angle, distance, timestamp), ...]

        # 状态估计
        self.prev_pose = None
        self.v_avg = 0.0
        self.w_avg = 0.0
    
    def start(self):
        """启动导航系统"""
        print(f"📡 连接串口 {self.port} @ {self.baud}")
        self.receiver_thread.start()
        print("✅ 数据接收线程已启动")

    def stop(self):
        """停止导航系统"""
        try:
            self._send_command('x')
            time.sleep(0.2)
            
            # 询问是否保存地图
            print("\n" + "="*60)
            save_choice = input("是否保存地图？ (y/n): ").strip().lower()
            if save_choice == 'y':
                print("\n正在保存地图...")
                metadata = {
                    'scan_count': self.lidar_data.scan_count,
                    'navigation_mode': 'auto_astar_dwa',
                    'start_world': self.start_world,
                    'goal_world': self.goal_world
                }
                MapManager.save_map(
                    self.slam.data.grid_map,
                    self.slam.data.robot_trajectory,
                    metadata=metadata,
                    filename_prefix='auto_nav_astar_map'
                )
            print("="*60)
        except Exception as e:
            print(f"停止时出错: {e}")
        finally:
            print("导航已停止")
    
    def _send_command(self, cmd: str):
        """发送控制命令"""
        if cmd == self.last_command:
            return
        
        try:
            if len(cmd) == 1:
                self.serial.ser.write(cmd.encode())
            else:
                self.serial.ser.write((cmd + '\n').encode())
            
            self.last_command = cmd
            print(f"发送命令: {cmd}")
        except Exception as e:
            print(f"发送命令失败: {e}")
    
    def _update_slam(self, timeout: float = 0.05) -> List[Tuple[float, float, float]]:
        """更新SLAM地图并返回激光雷达点 [(angle, distance, timestamp), ...]"""
        lidar_points = []
        t_end = time.time() + timeout
        packet_count = 0
        current_time = time.time()
        
        while time.time() < t_end:
            try:
                packet = self.data_queue.get_nowait()
            except queue.Empty:
                break

            if not isinstance(packet, dict):
                if packet == 'SYNC':
                    self.lidar_data.scan_count += 1
                continue

            pkt_type = packet.get('type')
            if pkt_type == 'ODOM_ONLY':
                continue

            if pkt_type == 'LIDAR_ODOM':
                angle = packet.get('angle')
                distance = packet.get('distance')
                robot_x = packet.get('odom_x')
                robot_y = packet.get('odom_y')
                robot_theta = packet.get('odom_theta')
                
                if None in (angle, distance, robot_x, robot_y, robot_theta):
                    continue

                # 过滤距离
                if distance < 0.20 or distance > 1.5:
                    continue

                # 更新SLAM
                try:
                    self.slam.update_map(
                        angle, distance, robot_x, robot_y, robot_theta,
                        weight=5, skip_rotation_check=False
                    )
                    packet_count += 1
                except Exception as e:
                    if packet_count < 5:
                        print(f"SLAM更新出错: {e}")

                # 收集激光雷达点（相对于机器人的角度）
                if distance > 0.15:
                    lidar_points.append((angle, distance, current_time))

        # 更新缓存
        self.lidar_points_cache = lidar_points
        return lidar_points

    def _get_current_state(self) -> RobotState:
        """获取当前机器人状态"""
        x, y, theta = self.shared_odom.get()
        
        # 估算速度
        if self.prev_pose is not None:
            px, py, pth = self.prev_pose
            dt = 0.05
            v = math.sqrt((x - px)**2 + (y - py)**2) / dt
            w = angle_diff(theta, pth) / dt
            self.v_avg = 0.7 * self.v_avg + 0.3 * v
            self.w_avg = 0.7 * self.w_avg + 0.3 * w
        
        self.prev_pose = (x, y, theta)
        return RobotState(x, y, theta, self.v_avg, self.w_avg)
    
    def _is_at_goal(self, state: RobotState, goal: Tuple[float, float], threshold: float = 0.20) -> bool:
        """检查是否到达目标点"""
        distance = math.sqrt((state.x - goal[0])**2 + (state.y - goal[1])**2)
        return distance < threshold
    
    def _execute_180_turn(self):
        """执行180度转向"""
        print("执行180度转向...")
        
        # 执行两次90度左转来完成180度转向
        for i in range(2):
            print(f"  转向 {i+1}/2...")
            self._send_command('A')
            time.sleep(1.0)  # 等待转向完成
            self._send_command('x')
            time.sleep(0.3)
        
        # 重置PID控制器
        self.wall_follower.reset_pid()
        print("180度转向完成")
    
    def run(self):
        """运行导航主循环 - 墙跟随模式"""
        self.start()
        
        try:
            print("等待初始化...")
            time.sleep(2.0)
            
            print("\n" + "="*60)
            print("🚗 开始墙跟随导航任务")
            print(f"   起点: {self.start_world}")
            print(f"   终点: {self.goal_world}")
            print(f"   目标墙距: {self.wall_distance*100:.0f}cm")
            print("="*60 + "\n")
            
            # 阶段1：前往终点（右墙跟随）
            print("📍 阶段1: 前往终点（沿右墙35cm）")
            self.navigation_phase = 'going'
            self._wall_following_phase(side='right', goal=self.goal_world)
            
            # 检查是否到达终点
            state = self._get_current_state()
            if self._is_at_goal(state, self.goal_world, threshold=0.25):
                print("\n✅ 已到达终点！")
                self._send_command('x')
                time.sleep(0.5)
                
                # 执行180度转向
                self._execute_180_turn()
                
                # 阶段2：返回起点（左墙跟随）
                print("\n📍 阶段2: 返回起点（沿左墙35cm）")
                self.navigation_phase = 'returning'
                self.returning = True
                self._wall_following_phase(side='left', goal=self.start_world)
                
                # 检查是否回到起点
                state = self._get_current_state()
                if self._is_at_goal(state, self.start_world, threshold=0.25):
                    print("\n✅ 已返回起点！")
                    self._send_command('x')
                else:
                    print(f"\n⚠️ 未完全返回起点，当前位置: ({state.x:.2f}, {state.y:.2f})")
            else:
                print(f"\n⚠️ 未到达终点，当前位置: ({state.x:.2f}, {state.y:.2f})")
            
            print("\n🎉 导航任务完成！")
            
        except KeyboardInterrupt:
            print("\n\n⚠️ 手动停止")
        except Exception as e:
            print(f"\n❌ 导航出错: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.stop()
    
    def _wall_following_phase(self, side: str, goal: Tuple[float, float]):
        """
        墙跟随阶段
        
        Args:
            side: 'right' 或 'left'，跟随哪侧的墙
            goal: 目标位置
        """
        last_status_time = time.time()
        status_interval = 1.0
        control_dt = 0.1
        
        side_cn = "右" if side == 'right' else "左"
        
        while True:
            # 更新SLAM和激光雷达数据
            lidar_points = self._update_slam(timeout=0.05)
            
            # 获取当前状态
            state = self._get_current_state()
            
            # 检查是否到达目标
            if self._is_at_goal(state, goal, threshold=0.25):
                print(f"\n✅ 到达目标位置！")
                self._send_command('x')
                break
            
            # 状态报告
            if time.time() - last_status_time > status_interval:
                distance_to_goal = math.sqrt((state.x - goal[0])**2 + (state.y - goal[1])**2)
                print(f"位置: ({state.x:.2f}, {state.y:.2f})m | "
                      f"朝向: {state.theta_deg:.1f}° | "
                      f"距目标: {distance_to_goal:.2f}m")
                last_status_time = time.time()
            
            # 检查前方障碍物
            if self.wall_follower.check_forward_obstacle(lidar_points):
                print(f"⚠️ 前方检测到障碍物，执行转向")
                # 根据跟随侧决定转向方向
                turn_cmd = 'A' if side == 'right' else 'D'
                self._send_command(turn_cmd)
                time.sleep(1.0)
                continue
            
            # 获取墙壁距离
            wall_distance, wall_angle = self.wall_follower.get_wall_distance_and_angle(
                lidar_points, side=side
            )
            
            if wall_distance is None:
                # 未检测到墙壁，继续前进并尝试寻找墙壁
                print(f"⚠️ 未检测到{side_cn}侧墙壁，继续前进")
                self._send_command('W')
                time.sleep(control_dt)
                continue
            
            # 显示墙距信息（每2秒显示一次详细信息）
            if time.time() - last_status_time > 2.0:
                print(f"  {side_cn}墙距: {wall_distance*100:.1f}cm (目标: {self.wall_distance*100:.0f}cm)")
            
            # 计算控制命令
            cmd = self.wall_follower.calculate_control(
                wall_distance, side=side, dt=control_dt
            )
            
            # 执行控制命令
            self._send_command(cmd)
            time.sleep(control_dt)
        
        print(f"{side_cn}墙跟随阶段完成")


# ========== 命令行参数 ==========

def parse_args():
    parser = argparse.ArgumentParser(description="自动导航系统 - 墙跟随算法")
    parser.add_argument('--port', type=str, required=True, help='串口，如COM7')
    parser.add_argument('--baud', type=int, default=115200, help='波特率')
    parser.add_argument('--start', type=float, nargs=2, default=None, metavar=('X', 'Y'),
                       help='起点坐标(米)，默认 (0.0, 0.0)')
    parser.add_argument('--goal', type=float, nargs=2, default=None, metavar=('X', 'Y'),
                       help='终点坐标(米)，默认 (1.05, 2.45) 即 105cm, 245cm')
    parser.add_argument('--wall-distance', type=float, default=0.35,
                       help='目标墙距(米)，默认0.35m (35cm)')
    return parser.parse_args()


def main():
    args = parse_args()
    
    start_world = tuple(args.start) if args.start else (0.0, 0.0)
    goal_world = tuple(args.goal) if args.goal else (1.05, 2.45)

    print("=" * 60)
    print("  自动导航系统 - 墙跟随算法")
    print("=" * 60)
    print(f"\n串口: {args.port} @ {args.baud}")
    print(f"地图尺寸: 280cm x 280cm (2.8m x 2.8m)")
    print(f"起点: {start_world} (0cm, 0cm)")
    print(f"终点: {goal_world} (105cm, 245cm)")
    print(f"目标墙距: {args.wall_distance*100:.0f}cm")
    print(f"\n导航策略:")
    print(f"  阶段1: 从起点到终点 - 沿右墙保持{args.wall_distance*100:.0f}cm")
    print(f"  阶段2: 执行180度转向")
    print(f"  阶段3: 从终点返回起点 - 沿左墙保持{args.wall_distance*100:.0f}cm")
    print(f"\nSLAM: {MAP_SIZE}x{MAP_SIZE}栅格 @ {MAP_RESOLUTION}m分辨率")
    print("=" * 60)
    print()

    navigator = AutoNavigator(
        port=args.port,
        baud=args.baud,
        start_world=start_world,
        goal_world=goal_world,
        wall_distance=args.wall_distance
    )
    
    navigator.run()


if __name__ == '__main__':
    main()