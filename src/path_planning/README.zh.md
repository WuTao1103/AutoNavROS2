# 路径规划包 (Path Planning Package) - 中文说明

这是一个用于室内自主TurtleBot导航的ROS2路径规划包，实现了基于A*算法的路径搜索和Pure Pursuit路径跟踪控制，支持无GPS环境下的自主导航。

## 🚀 快速开始

### 仿真环境测试
```bash
# 1. 构建包
cd /home/rog/AutoNavROS2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select path_planning
source install/setup.bash

# 2. 启动仿真环境
ros2 launch path_planning simulation_test.launch.py

# 3. 在新终端启动可视化
ros2 run path_planning simple_visualizer.py

# 4. 测试路径规划服务
ros2 service call /plan_path path_planning/srv/PlanPath "start: {header: {frame_id: 'map'}, pose: {position: {x: -2.0, y: -2.0, z: 0.0}, orientation: {w: 1.0}}}, goal: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}, tolerance: 0.2"
```

## 项目概述

本项目实现了无GPS环境下的室内自主导航系统。路径规划包负责在已知地图上规划从起点到终点的最优路径，并提供路径跟踪控制功能。

### 三层开发架构
- **Level 1**: 基础路径跟踪和移动控制 ✅
- **Level 2**: 与SLAM集成的2D地图路径规划 🚧
- **Level 3**: 高级3D语义地图和多机器人协调

## 功能特性

### 已实现功能 ✅
- **A*路径规划算法** - 基于占据栅格地图的最优路径搜索
- **Pure Pursuit控制器** - 平滑的路径跟踪控制
- **路径规划服务** - ROS2服务接口，支持外部路径规划请求
- **地图仿真器** - 多种地图类型的仿真环境（房间、迷宫、办公室）
- **可视化工具** - 实时显示地图和规划路径
- **障碍物膨胀** - 考虑机器人半径的安全路径规划
- **路径平滑** - 减少路径锯齿，提高跟踪性能
- **测试客户端** - 自动化测试不同路径规划场景

### 计划功能 🚧
- **动态重规划** - 检测到新障碍物时重新计算路径
- **与SLAM集成** - 接入真实的SLAM地图数据
- **多机器人协调** - 支持多机器人路径规划

## 文件夹结构

```
src/path_planning/
├── CMakeLists.txt                     # CMake构建配置
├── package.xml                        # ROS2包依赖定义
├── README.md                          # 英文说明文档
├── README.zh.md                       # 中文说明文档（本文件）
├── config/                            # 配置文件夹
│   └── path_planning_params.yaml      # 路径规划参数配置
├── launch/                            # 启动文件夹
│   ├── path_planning.launch.py        # 基础路径规划启动文件
│   └── simulation_test.launch.py      # 仿真测试启动文件
├── scripts/                           # 可执行脚本文件夹
│   ├── astar_planner.py              # A*路径规划算法实现
│   ├── path_planning_service.py      # 路径规划服务节点
│   ├── pure_pursuit_controller.py   # Pure Pursuit路径跟踪控制器
│   ├── simple_map_simulator.py      # 地图仿真器
│   ├── simple_path_publisher.py     # 测试路径发布器
│   ├── simple_visualizer.py         # 可视化工具
│   ├── test_path_planning_client.py # 路径规划服务测试客户端
│   └── test_tf_publisher.py         # TF变换测试发布器
├── srv/                              # 服务定义文件夹
│   └── PlanPath.srv                  # 路径规划服务定义
└── path_planning/                    # Python包文件夹
    └── __init__.py                   # Python包初始化文件
```

## 主要组件说明

### 1. 核心算法 (`scripts/`)

#### `astar_planner.py` - A*路径规划器
- **功能**: 基于A*算法在占据栅格地图上寻找最优路径
- **特性**:
  - 支持8连通和4连通移动
  - 障碍物膨胀（考虑机器人半径）
  - 可配置启发函数权重
  - 路径平滑优化
- **输入**: 起点和终点的世界坐标
- **输出**: 路径点序列

#### `pure_pursuit_controller.py` - Pure Pursuit控制器
- **功能**: 实现Pure Pursuit算法进行路径跟踪
- **特性**:
  - 可配置前视距离
  - 自适应线速度（转弯时减速）
  - TF坐标变换支持
  - 目标到达检测
- **输入**: 路径消息 (`nav_msgs/Path`)
- **输出**: 速度控制命令 (`geometry_msgs/Twist`)

#### `path_planning_service.py` - 路径规划服务
- **功能**: 提供ROS2服务接口，整合A*规划器
- **服务名**: `/plan_path`
- **服务类型**: `path_planning/PlanPath`
- **特性**:
  - 输入验证和错误处理
  - 规划时间限制
  - 路径自动发布用于可视化
  - 紧急停止功能

### 2. 仿真和测试 (`scripts/`)

#### `simple_map_simulator.py` - 地图仿真器
- **功能**: 生成不同类型的测试地图
- **地图类型**:
  - `empty` - 空旷环境（仅边界墙）
  - `room_with_obstacles` - 房间内有各种障碍物
  - `maze` - 简单迷宫结构
  - `office` - 办公室布局
- **输出**: 占据栅格地图 (`nav_msgs/OccupancyGrid`)

#### `simple_visualizer.py` - 可视化工具
- **功能**: 实时显示地图和路径规划结果
- **特性**:
  - matplotlib绘图界面
  - 实时数据更新
  - 地图和路径同步显示
- **依赖**: `matplotlib`, `numpy`

#### `test_path_planning_client.py` - 测试客户端
- **功能**: 自动测试路径规划服务
- **测试场景**:
  - 对角线移动
  - 十字穿越
  - 直线路径
  - 返回原点
- **特性**: 定时自动测试，结果日志记录

### 3. 配置文件 (`config/`)

#### `path_planning_params.yaml` - 参数配置文件
包含所有节点的配置参数：
- **Pure Pursuit参数**: 前视距离、速度限制、目标容差
- **A*规划参数**: 启发函数权重、对角移动、障碍物膨胀
- **服务配置**: 话题名称、超时时间
- **测试参数**: 测试间隔、坐标框架

### 4. 启动文件 (`launch/`)

#### `path_planning.launch.py` - 基础启动文件
启动核心路径规划组件：
- Pure Pursuit控制器
- 测试路径发布器
- 路径规划服务

#### `simulation_test.launch.py` - 仿真测试启动文件
启动完整仿真环境：
- 地图仿真器
- 路径规划服务
- Pure Pursuit控制器
- （可选）自动测试客户端

### 5. 服务定义 (`srv/`)

#### `PlanPath.srv` - 路径规划服务定义
```
# 请求
geometry_msgs/PoseStamped start    # 起点位姿
geometry_msgs/PoseStamped goal     # 终点位姿
float64 tolerance                  # 目标容差（可选）
---
# 响应
nav_msgs/Path path                 # 规划的路径
bool success                       # 成功标志
string message                     # 状态消息
float64 planning_time              # 规划耗时
```

## 安装和使用

### 1. 依赖要求
- ROS2 (Humble/Iron/Jazzy)
- Python 3.8+
- numpy
- matplotlib (用于可视化)
- Navigation2消息包

### 2. 构建包
```bash
# 从工作空间根目录
colcon build --packages-select path_planning

# 加载环境
source install/setup.bash
```

### 3. 基础使用

#### 启动仿真环境
```bash
# 启动完整仿真（包含地图和路径规划服务）
ros2 launch path_planning simulation_test.launch.py

# 选择不同地图类型
ros2 launch path_planning simulation_test.launch.py map_type:=maze
ros2 launch path_planning simulation_test.launch.py map_type:=office
```

#### 启动可视化
```bash
# 在新终端中启动可视化工具
ros2 run path_planning simple_visualizer.py
```

#### 手动测试路径规划
```bash
# 调用路径规划服务
ros2 service call /plan_path path_planning/srv/PlanPath "
start:
  header: {frame_id: 'map'}
  pose:
    position: {x: -2.0, y: -2.0, z: 0.0}
    orientation: {w: 1.0}
goal:
  header: {frame_id: 'map'}
  pose:
    position: {x: 2.0, y: 2.0, z: 0.0}
    orientation: {w: 1.0}
tolerance: 0.2
"
```

### 4. 运行自动测试
```bash
# 启动自动测试客户端
ros2 run path_planning test_path_planning_client.py
```

## 参数配置

### Pure Pursuit控制器参数
- `lookahead_distance`: 前视距离（米）
- `max_linear_velocity`: 最大线速度（米/秒）
- `max_angular_velocity`: 最大角速度（弧度/秒）
- `goal_tolerance`: 目标容差（米）

### A*规划器参数
- `allow_diagonal`: 是否允许对角移动
- `heuristic_weight`: 启发函数权重
- `inflation_radius`: 障碍物膨胀半径（米）
- `robot_radius`: 机器人半径（米）

### 地图仿真器参数
- `map_width/height`: 地图尺寸（栅格数）
- `resolution`: 地图分辨率（米/栅格）
- `origin_x/y`: 地图原点（米）
- `map_type`: 地图类型

## 开发团队

- **负责人**: Tao Wu

## 开发时间线

- **Level 1** (Nov 10-23): 基础路径跟踪和移动控制 ✅
- **Level 2** (Nov 24-Dec 14): SLAM集成和基于地图的路径规划 🚧
- **Level 3** (Dec 9-14): 高级功能和优化

## 故障排除

### 常见问题

1. **构建失败**
   ```bash
   # 清理构建缓存
   rm -rf build/ install/
   colcon build --packages-select path_planning
   ```

2. **服务调用失败**
   ```bash
   # 检查服务状态
   ros2 service list | grep plan_path
   ros2 service type /plan_path
   ```

3. **可视化无法显示**
   ```bash
   # 安装matplotlib
   pip3 install matplotlib
   ```

4. **没有地图数据**
   ```bash
   # 确保地图仿真器运行
   ros2 topic echo /map --once
   ```

### 调试技巧

1. **查看话题数据**
   ```bash
   ros2 topic list
   ros2 topic echo /map
   ros2 topic echo /path
   ```

2. **检查节点状态**
   ```bash
   ros2 node list
   ros2 node info /path_planning_service
   ```

3. **查看日志**
   ```bash
   ros2 launch path_planning simulation_test.launch.py --ros-args --log-level DEBUG
   ```

## 可视化和测试指南

### 🎨 可视化方法

#### 方法1：RViz2可视化 (推荐)
```bash
# 新终端启动RViz2
cd /home/rog/AutoNavROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
rviz2
```

**RViz2设置步骤：**
1. 设置 **Fixed Frame** 为 `map`
2. **Add** → **By topic** → `/map` → **Map** (显示地图)
3. **Add** → **By topic** → `/path` → **Path** (显示路径)
4. 可选：**Add** → **TF** (显示坐标系)

#### 方法2：修复matplotlib可视化工具
```bash
# 安装matplotlib依赖
pip3 install matplotlib --break-system-packages

# 设置显示环境
export DISPLAY=:0
export MPLBACKEND=TkAgg

# 启动自定义可视化工具
ros2 run path_planning simple_visualizer.py
```

#### 方法3：命令行查看地图数据
```bash
# 显示地图基本信息
ros2 topic echo /map --once | grep -A10 "info:"

# 统计地图数据分布
ros2 topic echo /map --once | grep -o "100" | wc -l  # 障碍物数量
ros2 topic echo /map --once | grep -o "0" | wc -l    # 自由空间数量
```

### 🧪 完整测试流程

#### 测试1：基础功能验证
```bash
# 终端1：启动仿真环境
cd /home/rog/AutoNavROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch path_planning simulation_test.launch.py map_type:=empty

# 终端2：系统状态检查
ros2 node list                    # 检查节点
ros2 service list | grep plan_path # 检查服务
ros2 topic echo /map --once       # 检查地图

# 终端3：自动测试
ros2 run path_planning test_path_planning_client.py
```

**预期结果：**
- 4个测试用例全部通过 ✅
- 规划时间 < 0.02s
- 路径点数量合理 (2-10个)

#### 测试2：不同地图类型
```bash
# 空地图测试 (基础验证)
ros2 launch path_planning simulation_test.launch.py map_type:=empty

# 迷宫地图测试 (复杂路径)
ros2 launch path_planning simulation_test.launch.py map_type:=maze

# 办公室地图测试 (现实场景)
ros2 launch path_planning simulation_test.launch.py map_type:=office

# 障碍物房间测试 (如果修复)
ros2 launch path_planning simulation_test.launch.py map_type:=room_with_obstacles
```

#### 测试3：手动路径规划
```bash
# 测试简单路径
ros2 service call /plan_path path_planning/srv/PlanPath "{
  start: {header: {frame_id: map}, pose: {position: {x: -2.0, y: -2.0, z: 0.0}, orientation: {w: 1.0}}},
  goal: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}},
  tolerance: 0.2
}"

# 测试复杂路径 (跨越障碍物)
ros2 service call /plan_path path_planning/srv/PlanPath "{
  start: {header: {frame_id: map}, pose: {position: {x: -4.0, y: -4.0, z: 0.0}, orientation: {w: 1.0}}},
  goal: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 4.0, z: 0.0}, orientation: {w: 1.0}}},
  tolerance: 0.1
}"
```

#### 测试4：性能监控
```bash
# 监控路径发布频率
ros2 topic hz /path

# 监控控制命令频率
ros2 topic hz /cmd_vel

# 查看节点资源使用
ros2 node info /path_planning_service

# 实时日志查看
ros2 topic echo /rosout | grep path_planning
```

### 📊 测试验证清单

#### ✅ 基础功能测试
- [ ] 所有节点正常启动
- [ ] 地图正确发布 (`/map` 话题有数据)
- [ ] 路径规划服务可用 (`/plan_path` 可调用)
- [ ] 自动测试4个用例全部通过
- [ ] A*算法规划时间 < 50ms

#### ✅ 地图测试
- [ ] `empty` 地图：大部分为自由空间(0)，边界为障碍物(100)
- [ ] `maze` 地图：有通道和墙壁的迷宫结构
- [ ] `office` 地图：房间和走廊的办公室布局
- [ ] 地图分辨率：0.1米/像素，大小100x100

#### ✅ 路径质量测试
- [ ] 路径避开障碍物
- [ ] 路径相对平滑（经过平滑算法）
- [ ] 起终点正确连接
- [ ] 无效路径请求正确拒绝

#### ✅ 可视化测试
- [ ] RViz2正确显示地图和路径
- [ ] 路径更新实时反映在可视化中
- [ ] 地图颜色正确（黑色=障碍物，白色=自由空间）

### 🐛 常见测试问题

#### 问题1：路径规划失败 "Failed to find a valid path"
**原因：** 起点或终点在障碍物中，或无可行路径
**解决：**
```bash
# 检查地图数据
ros2 topic echo /map --once | grep -A20 "data:" | head -50

# 尝试更安全的起终点
ros2 service call /plan_path path_planning/srv/PlanPath "{...}"  # 使用地图中心区域坐标
```

#### 问题2：可视化工具崩溃
**原因：** matplotlib显示问题
**解决：**
```bash
# 使用RViz2替代
rviz2

# 或设置无头模式
export MPLBACKEND=Agg
```

#### 问题3：节点启动失败
**原因：** 环境未正确加载
**解决：**
```bash
# 重新加载环境
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 检查包是否正确安装
ros2 pkg list | grep path_planning
```



## 许可证

MIT License - 详见项目根目录

## 贡献

1. 遵循ROS2最佳实践和约定
2. 在硬件部署前先在仿真中测试所有更改
3. 为新功能更新文档
4. 与SLAM团队协调接口变更

---

**最后更新**: 2024年11月30日
**版本**: Level 2 - A*路径规划和仿真环境完成
**测试状态**: ✅ 通过 - 仿真环境运行正常，路径规划服务可用

## 开发团队

**负责人**: 陶武

## 技术支持

如有问题请查看：
1. ROS2官方文档: https://docs.ros.org/
2. Navigation2文档: https://navigation.ros.org/
3. 项目GitHub Issues

---

**最后更新**: 2024年11月19日
**版本**: Level 1 - Pure Pursuit实现（已完成并验证）
**测试状态**: ✅ 通过 - 控制器以30Hz稳定运行



终端1：启动仿真环境

  cd /home/rog/AutoNavROS2
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  ros2 launch path_planning simulation_test.launch.py
  map_type:=room_with_obstacles

  终端2：启动可视化工具

  cd /home/rog/AutoNavROS2
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  ros2 run path_planning simple_visualizer.py

  终端3：测试路径规划服务

  cd /home/rog/AutoNavROS2
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash

  # 手动测试路径规划
  ros2 service call /plan_path path_planning/srv/PlanPath "
  start:
    header: {frame_id: 'map'}
    pose:
      position: {x: -2.0, y: -2.0, z: 0.0}
      orientation: {w: 1.0}
  goal:
    header: {frame_id: 'map'}
    pose:
      position: {x: 2.0, y: 2.0, z: 0.0}
      orientation: {w: 1.0}
  tolerance: 0.2
  "

  可选：终端4：自动测试

  cd /home/rog/AutoNavROS2
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  ros2 run path_planning test_path_planning_client.py

  📋 验证清单

  终端1应该显示：
  - "Simple Map Simulator started"
  - "Path Planning Service ready"
  - "Pure Pursuit Controller initialized"

  终端2应该显示：
  - matplotlib窗口显示地图和路径

  终端3应该返回：
  - success: true
  - 规划的路径点数量

  试试看，如果遇到问题告诉我！

### 🔧 交互式路径规划测试

#### 简化测试方法（推荐）
```bash
# 步骤1：准备环境
cd /home/rog/AutoNavROS2
source install/setup.bash

# 步骤2：直接启动交互式规划器
ros2 run path_planning interactive_planner.py
```

**交互式规划器特性：**
- 🎯 **图形界面**：用户友好的命令行界面，带有清晰的提示
- 🗺️ **实时地图**：自动启动地图仿真环境
- 📍 **灵活输入**：可以手动输入任意起点和终点坐标
- ✅ **即时反馈**：显示路径规划结果和详细统计信息
- 🔄 **连续测试**：可以进行多次路径规划而无需重启

#### 使用示例：
```
============================================================
🗺️  INTERACTIVE PATH PLANNER
============================================================
Map range: -5.0 to 5.0 meters (X and Y)
Safe areas are usually away from (0,0) center
Try coordinates like: -3.5, -2.0, 2.0, 3.5
============================================================

📍 SET START POINT:
Start X coordinate (meters): -3.5
Start Y coordinate (meters): -3.5

🎯 SET GOAL POINT:
Goal X coordinate (meters): 3.5
Goal Y coordinate (meters): 3.5
Tolerance (0.1-0.5 meters, default 0.2): 0.3

🚀 Planning path from (-3.5, -3.5) to (3.5, 3.5)
Planning...

==================================================
✅ PATH PLANNING SUCCESSFUL!
📊 Waypoints: 22
⏱️  Planning time: 0.040 seconds
💬 Message: Path planned successfully with 22 waypoints

🛣️  PATH WAYPOINTS:
   🚩 Start: (-3.60, -3.60)
   📍 Point 1: (-3.40, -3.20)
   [... 其他路径点 ...]
   🎯 Goal:  (3.40, 3.40)

📏 Total path distance: 10.38 meters
==================================================
```

#### 📊 改进的碰撞检测（最新版本）
**算法增强：**
- ✅ **密集采样**：双倍密度的路径检查点
- ✅ **细障碍物检测**：可以检测单像素宽的障碍物
- ✅ **安全边距**：自动检测狭窄通道并避免穿越
- ✅ **平衡策略**：在路径安全性和可达性之间取得平衡

**测试验证：**
- 成功避免穿越细障碍物 ✅
- 在开阔区域正常规划路径 ✅
- 正确拒绝不安全的路径 ✅
- 保持合理的规划成功率 ✅



决方案2: 手动启动 ros_gz_bridge
bash# 安装 ros_gz_bridge (如果还没安装)
sudo apt install ros-jazzy-ros-gz-bridge -y

### 方法1: 使用自动同步节点（推荐）

`gazebo_map_sync.py` 节点会自动将 `simple_map_simulator` 生成的地图障碍物同步到Gazebo仿真环境中。

#### 完整启动流程：

```bash
# 终端1: 启动Gazebo空白世界
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py

# 终端2: 启动地图同步和路径规划（自动同步障碍物）
cd /home/rog/AutoNavROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch path_planning gazebo_simulation.launch.py map_type:=room_with_obstacles
```

这个launch文件会自动：
1. 启动 `simple_map_simulator` 生成地图
2. 启动 `gazebo_map_sync` 将障碍物同步到Gazebo
3. 启动路径规划服务和控制器

#### 手动启动各个组件：

```bash
# 终端1: 启动Gazebo
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py

# 终端2: 启动地图仿真器
cd /home/rog/AutoNavROS2
source install/setup.bash
ros2 run path_planning simple_map_simulator.py --ros-args -p map_type:=room_with_obstacles

# 终端3: 启动地图同步节点（将障碍物spawn到Gazebo）
ros2 run path_planning gazebo_map_sync.py

# 终端4: 路径规划服务
ros2 run path_planning path_planning_service.py

# 终端5: Pure Pursuit控制器
ros2 run path_planning pure_pursuit_controller.py
```

### 方法2: 使用ros_gz_bridge（仅控制命令）

如果只需要控制命令桥接，不需要同步障碍物：

# 终端5: 测试
cd /home/rog/AutoNavROS2&& source install/setup.bash
ros2 run path_planning interactive_planner.py
```



解决方案2: 手动启动 ros_gz_bridge
bash# 安装 ros_gz_bridge (如果还没安装)
sudo apt install ros-jazzy-ros-gz-bridge -y

# 启动桥接 - ROS2 Twist 到 Gazebo
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist


