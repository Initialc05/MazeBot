# MazeBot 上位机符号体系参考手册
> **版本**: V4.1  
> **日期**: 2025-10-14  
> **目的**: 统一算法开发和团队沟通的符号系统

---

## ⚡ 快速参考 - 显示坐标系变量（算法开发专用）

> **🎯 核心原则**: 后续算法开发**只使用显示坐标系变量**，无需关心IMU内部坐标系！

### 📍 **机器人状态变量**（实时更新）

| 符号 | Python变量 | 单位 | 含义 | 获取方式 |
|------|-----------|------|------|----------|
| **x** | `display_x` | 米 | X坐标（右为正） | `x, y, θ = get_robot_pose_display()` |
| **y** | `display_y` | 米 | Y坐标（上为正） | 同上 |
| **θ** | `robot_theta` | 度 | 朝向角（逆时针为正，0°=X轴正方向） | 同上 |
| **(x_g, y_g)** | `robot_pos` | 栅格 | 栅格坐标 | `self.lidar_data.robot_pos` |

### 🗺️ **地图数据**

| 符号 | Python变量 | 类型 | 含义 |
|------|-----------|------|------|
| **G[i,j]** | `grid_map[i,j]` | int8 | 栅格占用值（-10~60） |
| **M** | `MAP_SIZE` | 常量 | 地图尺寸 = 500格 |
| **r** | `MAP_RESOLUTION` | 常量 | 栅格分辨率 = 0.05m/格 |

**占用值含义**:
- `G[i,j] = 0` → 未知
- `G[i,j] < 0` → 自由空间（可通行）
- `G[i,j] > 0` → 障碍物（值越大越可靠）

### 📊 **雷达扫描数据**

| 符号 | Python变量 | 单位 | 含义 |
|------|-----------|------|------|
| **α** | `angles[i]` | 度 | 第i个点的角度（0-360°） |
| **d** | `distances[i]` | 米 | 第i个点的距离（0.1-3.0m） |
| **n** | `len(angles)` | 个 | 当前扫描点数 |

**访问方式**:
```python
self.lidar_data.angles      # 角度列表
self.lidar_data.distances   # 距离列表
self.lidar_data.qualities   # 质量列表
```

### 🔧 **常用函数**

```python
# 获取机器人位置（显示坐标系）
def get_robot_pose_display():
    x_imu, y_imu, theta = self.shared_odom.get()
    x = -y_imu  # 显示X = -IMU_Y
    y = x_imu   # 显示Y = IMU_X
    return x, y, theta

# 世界坐标 → 栅格坐标
def world_to_grid(x, y):
    x_grid = int(250 + (-y) / 0.05)
    y_grid = int(250 + x / 0.05)
    return x_grid, y_grid

# 栅格坐标 → 世界坐标
def grid_to_world(x_grid, y_grid):
    y = -(x_grid - 250) * 0.05
    x = (y_grid - 250) * 0.05
    return x, y
```

### 🎮 **运动控制**

| 指令 | 功能 | 示例 |
|------|------|------|
| `F[cm]` | 前进N厘米 | `F50` = 前进50cm |
| `B[cm]` | 后退N厘米 | `B30` = 后退30cm |
| `L[deg]` | 左转N度 | `L90` = 左转90° |
| `R[deg]` | 右转N度 | `R45` = 右转45° |

**发送指令**:
```python
self.serial.ser.write(b'F50\n')  # 前进50cm
```

---

## 📐 坐标系定义

### 🌍 **显示坐标系**（统一标准，Status和地图都使用此坐标系）
```
        Y↑ (前进方向)
         |
         |
         |
    ----+----> X (右侧方向)
         |
       原点
```

- **X轴**: 水平向右为正 (m)
- **Y轴**: 竖直向上为正 (m)  
- **θ (Heading)**: 逆时针旋转为正，0°=X轴正方向 (度)
- **原点**: 机器人启动位置 (地图中心 = 栅格坐标 250, 250)

## 📊 扫描数据变量

### **Lidar扫描数据**

| 符号 | 变量名 | 单位 | 含义 | 范围 |
|------|--------|------|------|------|
| `α` | `angle` | 度 (°) | 雷达扫描角度 | 0-360° |
| `d` | `distance` | 米 (m) | 雷达测距 | 0.1-3.0m |
| `q` | `quality` | 无量纲 | 扫描质量 | 0-63 |
| `n_scans` | `scan_count` | 次 | 完整扫描圈数 | 整数 |
| `n_points` | `len(angles)` | 个 | 当前显示点数 | 整数 |

**扫描频率**:
- RPLidar C1: **10 Hz** (100ms/圈)
- 数据发送率: **2000-4000 packets/sec** (过滤后)

---

## 🗺️ 地图变量

### **栅格地图参数**

| 符号 | 变量名 | 值 | 单位 | 含义 |
|------|--------|----|------|------|
| `M` | `MAP_SIZE` | 500 | 栅格 | 地图尺寸 (500×500) |
| `r` | `MAP_RESOLUTION` | 0.05 | m/格 | 栅格分辨率 (5cm) |
| `d_max` | `MAX_RANGE` | 3.0 | m | 最大有效距离 |
| `(x_c, y_c)` | 地图中心 | (250, 250) | 栅格 | 原点位置 |

**地图覆盖范围**:
- X方向: -12.5m ~ +12.5m (500格 × 0.05m/格)
- Y方向: -12.5m ~ +12.5m

### **栅格地图数据**

| 符号 | 变量名 | 数据类型 | 值域 | 含义 |
|------|--------|----------|------|------|
| `G[i,j]` | `grid_map[i,j]` | int8 | -10 ~ 60 | 栅格占用状态 |

**占用状态定义**:
```python
G[i,j] = 0        # 未知区域
G[i,j] = -10~-1   # 自由空间（可通行）
G[i,j] = 1~60     # 障碍物（值越大越可靠）
```

### **障碍物统计**

| 符号 | 变量名 | 单位 | 含义 |
|------|--------|------|------|
| `N_obs` | `obstacle_cells` | 个 | 障碍物栅格数量 |
| `G_max` | `max_obstacle_val` | 无量纲 | 最大障碍物置信度 |

---

## 🚀 实时性能指标

### **数据通信**

| 符号 | 变量名 | 单位 | 含义 | 正常范围 |
|------|--------|------|------|----------|
| `Q` | `queue_size` | 个 | 接收队列长度 | < 1000 |
| `N_odom` | `odom_only_count` | 个 | 独立Odom包数 | ~250 Hz |
| `N_lidar` | `lidar_odom_count` | 个 | 雷达融合包数 | 2000-4000 Hz |
| `N_drop` | `drop_count` | 个 | 丢包数量 | 0 (理想) |

**数据包类型**:
```python
# 独立Odom包 (0xBB55): 11字节，250Hz
{
    'type': 'ODOM_ONLY',
    'odom_x': float,  # 米
    'odom_y': float,  # 米
    'odom_theta': float  # 度
}

# 雷达融合包 (0xAA55): 18字节，2000-4000Hz
{
    'type': 'LIDAR_ODOM',
    'angle': float,     # 度
    'distance': float,  # 米
    'quality': int,     # 0-63
    'odom_x': float,
    'odom_y': float,
    'odom_theta': float
}
```

---

## 🎮 运动控制指令

### **连续运动指令** (实时控制)

| 指令 | 功能 | 备注 |
|------|------|------|
| `⬆` | 前进 | 持续按下 |
| `⬇` | 后退 | 持续按下 |
| `⬅` | 左转 | 持续按下 |
| `➡` | 右转 | 持续按下 |
| `x` / `Space` | 急停 | 立即制动 |

### **精准运动指令** (闭环控制)

| 格式 | 示例 | 功能 | 参数范围 |
|------|------|------|----------|
| `F[cm]` | `F50` | 前进N厘米 | 10-200 cm |
| `B[cm]` | `B30` | 后退N厘米 | 10-200 cm |
| `L[deg]` | `L90` | 左转N度 | 1-180° |
| `R[deg]` | `R45` | 右转N度 | 1-180° |

---

## 📈 算法开发常用变量

### **Python访问路径**

```python
# 在 LidarVisualizer 类中
self.lidar_data.robot_pos          # (x_grid, y_grid) 栅格坐标
self.lidar_data.robot_theta        # θ 朝向角（度）
self.lidar_data.grid_map           # G[i,j] 栅格地图 (500×500)
self.lidar_data.robot_trajectory   # [(x1,y1), (x2,y2), ...] 轨迹

self.lidar_data.angles             # [α1, α2, ...] 雷达角度列表
self.lidar_data.distances          # [d1, d2, ...] 雷达距离列表
self.lidar_data.scan_count         # n_scans 扫描次数

self.shared_odom.get()             # (x_imu, y_imu, θ) IMU原始坐标
```

### **坐标转换函数**

```python
def imu_to_display(x_imu, y_imu, theta_imu):
    """IMU坐标系 → 显示坐标系"""
    x_display = -y_imu
    y_display = x_imu
    theta_display = theta_imu
    return x_display, y_display, theta_display

def world_to_grid(x_world, y_world):
    """世界坐标(米) → 栅格坐标"""
    x_grid = int(MAP_SIZE / 2 + (-y_world) / MAP_RESOLUTION)
    y_grid = int(MAP_SIZE / 2 + x_world / MAP_RESOLUTION)
    return x_grid, y_grid

def grid_to_world(x_grid, y_grid):
    """栅格坐标 → 世界坐标(米)"""
    y_world = -(x_grid - MAP_SIZE / 2) * MAP_RESOLUTION
    x_world = (y_grid - MAP_SIZE / 2) * MAP_RESOLUTION
    return x_world, y_world
```

---

## 🔧 配置参数

### **可视化性能**

```python
UPDATE_INTERVAL_MS = 6         # 更新间隔 (ms)
POINTS_PER_FRAME = 10000       # 每帧处理包数
MAX_DISPLAY_TIME = 0.1         # 显示时间窗口 (s)
```

### **运动过滤**

```python
USE_MOTION_FILTER = True       # 运动预测过滤
MOTION_THRESHOLD = 0.1         # 运动阈值 (m)
ANGLE_THRESHOLD = 10.0         # 角度阈值 (度)
```

### **多线程优化**

```python
USE_MULTITHREADING = True      # 多线程模式
USE_SEPARATE_ODOM_THREAD = True  # 独立Odom线程
```

---

## 📝 Status显示格式

```
Scans: n_scans | Points: n_points (t_span s) | Pos: (x, y)m | Heading: θ° | Obstacles: N_obs | Max: G_max | Q:queue Odom:N_odom Lidar:N_lidar Drop:N_drop
```

**示例**:
```
Scans: 150 | Points: 450/500 (0.10s) | Pos: (1.25, 0.80)m | Heading: 45.3° | Obstacles: 1250 | Max: 55 | Q:120 Odom:37500 Lidar:8000 Drop:0
```

---

## 🎯 算法开发示例

### **示例1: 获取当前机器人位置（显示坐标系）**

```python
def get_robot_pose_display():
    """获取机器人位置（显示坐标系）"""
    # 方法1: 从SharedOdomData（最实时）
    if USE_SEPARATE_ODOM_THREAD and self.shared_odom:
        x_imu, y_imu, theta = self.shared_odom.get()
        x = -y_imu  # 显示X = -IMU_Y
        y = x_imu   # 显示Y = IMU_X
        return x, y, theta
    
    # 方法2: 从栅格坐标反推
    x_grid, y_grid = self.lidar_data.robot_pos
    x = (y_grid - MAP_SIZE / 2) * MAP_RESOLUTION
    y = -(x_grid - MAP_SIZE / 2) * MAP_RESOLUTION
    theta = self.lidar_data.robot_theta
    return x, y, theta
```

### **示例2: 检测前方障碍物**

```python
def detect_obstacle_ahead(max_distance=0.5):
    """检测前方max_distance米内是否有障碍物"""
    x, y, theta = get_robot_pose_display()
    x_grid, y_grid = world_to_grid(x, y)
    
    # 计算前方检测点
    steps = int(max_distance / MAP_RESOLUTION)
    for step in range(1, steps + 1):
        check_x = x + step * MAP_RESOLUTION * np.cos(np.deg2rad(theta))
        check_y = y + step * MAP_RESOLUTION * np.sin(np.deg2rad(theta))
        check_x_grid, check_y_grid = world_to_grid(check_x, check_y)
        
        # 检查栅格占用状态
        if 0 <= check_x_grid < MAP_SIZE and 0 <= check_y_grid < MAP_SIZE:
            if self.lidar_data.grid_map[check_y_grid, check_x_grid] > 0:
                return True, step * MAP_RESOLUTION
    
    return False, max_distance
```

### **示例3: 计算到目标点的距离和角度**

```python
def get_target_relative_pose(target_x, target_y):
    """计算目标点相对机器人的距离和角度（显示坐标系）"""
    x, y, theta = get_robot_pose_display()
    
    # 相对位置
    dx = target_x - x
    dy = target_y - y
    
    # 距离
    distance = np.sqrt(dx**2 + dy**2)
    
    # 世界坐标系中的目标角度
    target_angle_world = np.rad2deg(np.atan2(dy, dx))
    
    # 相对于机器人朝向的角度差
    angle_diff = target_angle_world - theta
    
    # 归一化到 [-180, 180]
    while angle_diff > 180:
        angle_diff -= 360
    while angle_diff < -180:
        angle_diff += 360
    
    return distance, angle_diff
```

---

## 📚 术语对照表

| 中文 | 英文 | 符号 | 变量名 |
|------|------|------|--------|
| 位置 | Position | (x, y) | `display_x`, `display_y` |
| 朝向 | Heading | θ | `robot_theta` |
| 姿态 | Pose | (x, y, θ) | `robot_pose` |
| 栅格 | Grid | - | `grid` |
| 障碍物 | Obstacle | - | `obstacle` |
| 自由空间 | Free Space | - | `free` |
| 扫描 | Scan | - | `scan` |
| 里程计 | Odometry | - | `odom` |
| 轨迹 | Trajectory | - | `trajectory` |

---

## ⚠️ 重要约定

1. **所有对外接口使用显示坐标系**（Status、算法、通信）
2. **IMU坐标系仅内部使用**（接收解析后立即转换）
3. **角度统一使用度（°）**，计算时转换为弧度
4. **距离统一使用米（m）**，栅格单位为格
5. **坐标原点 = 机器人启动位置 = 地图中心(250, 250)**

---

## 🔗 相关文件

- **主程序**: `HOST/lidar_visualizer.py`
- **Arduino固件**: `im948_CMD.ino`, `LidarModule.h`, `MotorControl.h`
- **配置**: 修改 `lidar_visualizer.py` 第25-60行

---

**最后更新**: 2025-01-14  
**维护者**: MazeBot开发团队

