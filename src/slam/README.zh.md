# SLAM 包 - 中文使用说明

## 概述

这是一个为TurtleBot自主导航设计的简单2D SLAM系统，使用占用栅格建图技术从LiDAR数据创建地图。

### 主要功能
- **占用栅格建图器**: 从LiDAR扫描数据构建2D地图，使用对数几率占用栅格映射
- **简单里程计**: 提供航位推算定位（可选，当Gazebo里程计不可用时使用）
- **路径规划集成**: 与现有的path_planning包无缝协作

## 系统架构

```
激光雷达(/scan) ──► 占用栅格建图器 ──► 地图(/map) ──► 路径规划
                          ▲
里程计(/odom) ─────────┘

TF坐标系树:
map ──► odom ──► base_link ──► base_scan
```

## 使用方法

### 1. 仅运行SLAM建图
```bash
ros2 launch slam slam.launch.py
```

### 2. SLAM与路径规划集成
```bash
ros2 launch slam slam_with_planning.launch.py
```

### 3. 在Gazebo仿真中使用
```bash
# 首先启动Gazebo（TurtleBot3世界）
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 然后启动SLAM和路径规划
ros2 launch slam slam_with_planning.launch.py use_gazebo:=true
```

### 4. 无Gazebo测试模式（使用地图模拟器）
```bash
# 这将使用内置地图模拟器进行测试
ros2 launch slam slam_with_planning.launch.py use_gazebo:=false use_simple_odom:=true
```

## 启动参数说明

- `params_file`: SLAM参数文件路径（默认: config/slam_params.yaml）
- `use_simple_odom`: 使用内部里程计节点（默认: false）
- `use_gazebo`: 是否在Gazebo中运行（影响地图模拟）（默认: false）

## ROS话题

### 订阅的话题
- `/scan` (sensor_msgs/LaserScan): LiDAR数据输入
- `/odom` (nav_msgs/Odometry): 机器人里程计
- `/cmd_vel` (geometry_msgs/Twist): 速度命令（用于简单里程计）

### 发布的话题
- `/map` (nav_msgs/OccupancyGrid): 生成的占用栅格地图
- `/odom` (nav_msgs/Odometry): 机器人里程计（如果使用simple_odometry）

## 参数配置

### 占用栅格建图器参数
```yaml
occupancy_grid_mapper:
  ros__parameters:
    # 地图尺寸和分辨率
    map_width: 200              # 栅格数量 (20m x 20m，分辨率0.1m)
    map_height: 200             # 栅格数量
    map_resolution: 0.1         # 米/栅格
    map_origin_x: -10.0         # 地图原点x坐标（米）
    map_origin_y: -10.0         # 地图原点y坐标（米）

    # 占用概率
    hit_probability: 0.7        # 被占用栅格的概率
    miss_probability: 0.3       # 空闲栅格的概率
    occupied_threshold: 0.65    # 标记为占用的阈值
    free_threshold: 0.35        # 标记为空闲的阈值

    # 激光参数
    max_range: 10.0             # 使用的最大激光范围（米）
    min_range: 0.1              # 使用的最小激光范围（米）

    publish_rate: 2.0           # 地图发布频率（Hz）
```

### 简单里程计参数
```yaml
simple_odometry:
  ros__parameters:
    publish_rate: 50.0          # 里程计发布频率（Hz）
    wheel_base: 0.16            # 轮距（米，TurtleBot3规格）
    wheel_radius: 0.033         # 轮半径（米）
    use_cmd_vel_estimation: true # 使用cmd_vel进行运动估计
```

## 与路径规划的集成

SLAM系统在 `/map` 话题上发布地图，路径规划系统自动使用：

1. **SLAM从传感器数据构建地图**
2. **路径规划服务使用地图进行A*规划**
3. **纯跟踪控制器跟随规划的路径**
4. **机器人移动产生新的传感器数据，更新地图**

## 可视化

在RViz2中查看建图过程：

```bash
ros2 run rviz2 rviz2
```

添加以下显示器：
- **Map**: 话题 `/map`
- **LaserScan**: 话题 `/scan`
- **Path**: 话题 `/path`（与路径规划一起使用时）
- **TF**: 查看坐标系关系

将Fixed Frame设置为 `map`。

## 故障排除

### 常见问题

1. **地图没有显示**：检查LiDAR数据是否在 `/scan` 话题上发布
2. **建图质量差**：调整 `hit_probability` 和 `miss_probability` 参数
3. **TF错误**：确保静态变换正确发布
4. **路径规划不工作**：确保SLAM在启动路径规划前已发布地图

### 调试命令

```bash
# 检查话题
ros2 topic list
ros2 topic echo /map --once

# 检查TF树
ros2 run tf2_tools view_frames

# 检查节点状态
ros2 node list
ros2 node info /occupancy_grid_mapper

# 检查参数
ros2 param list /occupancy_grid_mapper
ros2 param get /occupancy_grid_mapper map_resolution
```

## 测试

包含与现有path_planning测试系统的集成。使用地图模拟器进行无硬件测试：

```bash
# 启动带模拟环境的SLAM
ros2 launch slam slam_with_planning.launch.py

# 在另一个终端测试路径规划
ros2 run path_planning test_path_planning_client.py
```

这将创建一个有障碍物的模拟环境并测试完整的SLAM + 路径规划流水线。

## 快速开始步骤

### 第一次使用：

1. **构建工作空间**
   ```bash
   colcon build --packages-select slam
   source install/setup.bash
   ```

2. **运行测试脚本**
   ```bash
   ./scripts/start_slam_demo.sh
   ```

3. **启动基本SLAM**
   ```bash
   ros2 launch slam slam.launch.py use_simple_odom:=true
   ```

4. **在另一个终端启动RViz2**
   ```bash
   ros2 run rviz2 rviz2
   ```
   - 添加Map显示器，话题设为 `/map`
   - Fixed Frame设为 `map`

5. **发布测试的cmd_vel命令让机器人移动**
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.2}" --once
   ```

### 与Gazebo配合使用：

1. **启动TurtleBot3 Gazebo世界**
   ```bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **启动SLAM和路径规划**
   ```bash
   ros2 launch slam slam_with_planning.launch.py use_gazebo:=true
   ```

3. **使用键盘控制机器人建图**
   ```bash
   ros2 run turtlebot3_teleop teleop_keyboard
   ```
