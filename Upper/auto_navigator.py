#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动导航系统 - 基于A*算法和DWA避障
支持90度和180度转向，能够规划最短路径从终点返回起点
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
        self.safety_distance = 0.1  # 10cm安全距离
        
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


# ========== 主控制器 ==========

class AutoNavigator:
    """自动导航控制器"""
    
    def __init__(self,
                 port: str,
                 baud: int = 115200,
                 start_world: Optional[Tuple[float, float]] = None,
                 goal_world: Optional[Tuple[float, float]] = None):
        self.port = port
        self.baud = baud
        self.start_world = start_world
        self.goal_world = goal_world
        
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

        # 规划器
        self.global_planner = None  # 将在运行时初始化
        self.local_planner = DWALocalPlanner()

        # 控制状态
        self.last_command = None
        self.current_path = []
        self.current_waypoint_index = 0
        self.returning = False
        self.original_goal = None

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
    
    def _update_slam(self, timeout: float = 0.05) -> List[Tuple[float, float]]:
        """更新SLAM地图并返回障碍物点"""
        obstacles_world = []
        t_end = time.time() + timeout
        packet_count = 0
        
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

                # 收集障碍物点
                if distance > 0.20:
                    ang_rad = math.radians(angle)
                    ox = robot_x + distance * math.cos(ang_rad + math.radians(robot_theta))
                    oy = robot_y + distance * math.sin(ang_rad + math.radians(robot_theta))
                    obstacles_world.append((ox, oy))

        return obstacles_world

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
    
    def _plan_global_path(self) -> List[Tuple[float, float]]:
        """规划全局路径"""
        if self.global_planner is None:
            self.global_planner = AStarPlanner(self.slam.data.grid_map)
        
        current_state = self._get_current_state()
        current_pos = (current_state.x, current_state.y)
        
        # 确定起点和终点
        start_pos = self.start_world if self.start_world else current_pos
        goal_pos = self.goal_world if self.goal_world else (2.0, 2.0)  # 默认目标
        
        print(f"规划路径: {start_pos} -> {goal_pos}")
        path = self.global_planner.plan(start_pos, goal_pos)
        
        if path:
            print(f"A*找到路径，共{len(path)}个点")
        else:
            print("A*未找到路径")
        
        return path
    
    def _plan_return_path(self) -> List[Tuple[float, float]]:
        """规划返回路径"""
        if self.global_planner is None:
            self.global_planner = AStarPlanner(self.slam.data.grid_map)
        
        current_state = self._get_current_state()
        current_pos = (current_state.x, current_state.y)
        
        # 返回起点
        start_pos = self.start_world if self.start_world else (0.0, 0.0)
        
        print(f"规划返回路径: {current_pos} -> {start_pos}")
        path = self.global_planner.plan(current_pos, start_pos)
        
        if path:
            print(f"返回路径找到，共{len(path)}个点")
        else:
            print("返回路径未找到")
        
        return path

    def _is_at_waypoint(self, state: RobotState, waypoint: Tuple[float, float], threshold: float = 0.3) -> bool:
        """检查是否到达路标点"""
        distance = math.sqrt((state.x - waypoint[0])**2 + (state.y - waypoint[1])**2)
        return distance < threshold
    
    def run(self):
        """运行导航主循环"""
        self.start()
        
        try:
            print("等待初始化...")
            time.sleep(1.0)
            
            # 探索阶段
            print("\n开始探索阶段...")
            self._explore_phase(duration=10.0)
            
            # 规划阶段
            print("\n开始路径规划...")
            self.current_path = self._plan_global_path()
            
            if not self.current_path:
                print("路径规划失败，继续探索...")
                self._explore_phase(duration=5.0)
                self.current_path = self._plan_global_path()
            
            if not self.current_path:
                print("无法找到路径，退出")
                return
            
            # 记录原始目标用于返回
            self.original_goal = self.goal_world
            
            # 导航阶段
            print("\n开始导航...")
            self._navigate_phase()
            
            # 返回阶段
            print("\n开始返回...")
            self.returning = True
            self.current_path = self._plan_return_path()
            self.current_waypoint_index = 0
            
            if self.current_path:
                self._navigate_phase()
            
            print("\n导航完成！")
            
        except KeyboardInterrupt:
            print("\n\n手动停止")
        except Exception as e:
            print(f"\n导航出错: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.stop()
    
    def _explore_phase(self, duration: float):
        """探索阶段"""
        start_time = time.time()
        last_turn_time = start_time
        turn_duration = 3.0
        last_status_time = start_time
        
        while time.time() - start_time < duration:
            # 更新SLAM
            obstacles = self._update_slam(timeout=0.02)
            
            # 状态报告
            if time.time() - last_status_time > 2.0:
                elapsed = int(time.time() - start_time)
                remaining = int(duration - elapsed)
                print(f"探索进度: {elapsed}/{int(duration)}秒 (剩余{remaining}秒) | 障碍物: {len(obstacles)}个")
                last_status_time = time.time()
            
            # 简单的探索策略：转圈观察
            if time.time() - last_turn_time > turn_duration:
                self._send_command('A')  # 左转
                last_turn_time = time.time()
                turn_duration = 2.0
            
            time.sleep(0.1)
        
        self._send_command('x')
        time.sleep(0.5)
        print("探索阶段完成")

    def _navigate_phase(self):
        """导航阶段"""
        last_status_time = time.time()
        last_replan_time = time.time()
        replan_interval = 5.0
        
        while self.current_waypoint_index < len(self.current_path):
            # 更新SLAM
            obstacles = self._update_slam(timeout=0.02)
            
            # 获取当前状态
            state = self._get_current_state()
            
            # 状态报告
            if time.time() - last_status_time > 1.0:
                progress = (self.current_waypoint_index / len(self.current_path)) * 100
                status = "返回" if self.returning else "前往"
                print(f"{status}进度: {self.current_waypoint_index}/{len(self.current_path)} ({progress:.0f}%) | "
                      f"位置: ({state.x:.2f}, {state.y:.2f})m | 朝向: {state.theta_deg:.1f}°")
                last_status_time = time.time()

            # 定期重新规划
            if time.time() - last_replan_time > replan_interval:
                print("重新规划路径...")
                if self.returning:
                    self.current_path = self._plan_return_path()
                else:
                    self.current_path = self._plan_global_path()
                self.current_waypoint_index = 0
                last_replan_time = time.time()
                if not self.current_path:
                    print("重新规划失败")
                    break

            # 检查是否到达当前路标点
            current_waypoint = self.current_path[self.current_waypoint_index]
            if self._is_at_waypoint(state, current_waypoint):
                print(f"到达路标点 {self.current_waypoint_index + 1}/{len(self.current_path)}")
                self.current_waypoint_index += 1
                self._send_command('x')
                time.sleep(0.3)
                continue

            # 使用DWA规划下一步动作
            cmd, duration = self.local_planner.plan(state, current_waypoint, obstacles)
            
            # 执行动作
            self._send_command(cmd)
            time.sleep(duration)
        
        self._send_command('x')
        print("导航阶段完成")


# ========== 命令行参数 ==========

def parse_args():
    parser = argparse.ArgumentParser(description="自动导航系统 - A* + DWA")
    parser.add_argument('--port', type=str, required=True, help='串口，如COM7')
    parser.add_argument('--baud', type=int, default=115200, help='波特率')
    parser.add_argument('--start', type=float, nargs=2, default=None, metavar=('X', 'Y'),
                       help='起点坐标(米)，如 --start 0.0 0.0')
    parser.add_argument('--goal', type=float, nargs=2, default=None, metavar=('X', 'Y'),
                       help='终点坐标(米)，如 --goal 2.0 2.0')
    return parser.parse_args()


def main():
    args = parse_args()
    
    start_world = tuple(args.start) if args.start else None
    goal_world = tuple(args.goal) if args.goal else None

    print("=" * 60)
    print("  自动导航系统 (A* + DWA)")
    print("=" * 60)
    print(f"\n串口: {args.port} @ {args.baud}")
    print(f"起点: {start_world if start_world else '自动检测'}")
    print(f"终点: {goal_world if goal_world else '默认(2.0, 2.0)'}")
    print(f"\n算法: A*全局规划 + DWA局部避障")
    print(f"转向: 支持90°和180°转向")
    print(f"SLAM: {MAP_SIZE}x{MAP_SIZE}栅格 @ {MAP_RESOLUTION}m分辨率")
    print("=" * 60)
    print()

    navigator = AutoNavigator(
        port=args.port,
        baud=args.baud,
        start_world=start_world,
        goal_world=goal_world
    )
    
    navigator.run()


if __name__ == '__main__':
    main()