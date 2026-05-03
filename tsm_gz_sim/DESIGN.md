# tsm_gz_sim 设计文档

## 概述

恭喜你解锁了全AI构建流哦

ROS 2 + Gazebo Harmonic 仿真模块，用于钢管直线度测量系统的仿真环境搭建与管道运动控制。

## 架构

```
tsm_gz_sim
├── Gazebo 仿真世界 (measure_world.sdf)
│   ├── UserCommands 插件        — 提供 set_pose service
│   ├── rgbd_1 相机 (+Y 侧)
│   └── rgbd_2 相机 (-Y 侧)
├── 管道模型 (straight_tube.sdf)
│   └── kinematic=true, gravity=false  — 可被 set_pose 移动
├── ROS 2 节点
│   ├── TswGzSimNode             — 管道运动控制
│   └── KeyboardNode             — 键盘输入
└── ros_gz_bridge                — 点云话题桥接
```

## 关键设计决策

### 仿真时间同步

所有节点启用 `use_sim_time: true`，使用 Gazebo 发布的 `/clock` 作为时间源。

**影响**：
- 点云、TF 时间戳与仿真时间一致，RViz 不会报 "No transform available"
- `TswGzSimNode` 使用 `rclcpp::create_timer`（节点时钟）而非 `create_wall_timer`（系统时钟），控制周期跟随仿真速度缩放

> `create_wall_timer`：始终按系统实时触发，仿真暂停时节点仍在发送指令。
> `rclcpp::create_timer`：按仿真时间触发，仿真暂停则节点暂停，速度 = `speed` (m/仿真秒)。

### 管道移动方式

**问题**：Gazebo 提供三种方式设置模型位姿：

| 方式 | 结果 |
|------|------|
| `static=true` | 完全固定，set_pose 无效 |
| `static=false`（纯物理） | 物理引擎每帧覆盖位置，set_pose 无效 |
| `kinematic=true` + `gravity=false` | 物理引擎不驱动，set_pose 有效 |

**结论**：管道 link 设置 `<kinematic>true</kinematic>` + `<gravity>false</gravity>`。

**控制接口**：使用 Gazebo transport service `/world/measure_world/set_pose`（非 blocking），通过 `gz::transport::Node::Request` 调用，携带模型名 `straight_tube` 和目标位置。

> `/world/measure_world/set_pose` topic 方式对 kinematic 模型无效，必须走 service。

### 相机布局

两台 RGBD 相机位于 YZ 平面，从 ±Y 侧以 45° 斜向下俯视管道：

| 模型 | 位置 | 朝向 (rpy) |
|------|------|-----------|
| rgbd_1 | (0, +1.0, 1.0) | (0, π/4, -π/2) |
| rgbd_2 | (0, -1.0, 1.0) | (0, π/4, +π/2) |

Gazebo 相机默认光轴为 +X。yaw=∓π/2 使光轴朝向管道中心，pitch=π/4 使光轴向下 45°。

### TF 树

相机为静态模型，使用 `static_transform_publisher` 发布固定 TF：

```
world
├── rgbd_1/camera_link/rgbd_1
└── rgbd_2/camera_link/rgbd_2
```

### 点云桥接

`ros_gz_bridge` 桥接 Gazebo → ROS 2：

- `/rgbd_1/points` : `gz.msgs.PointCloudPacked` → `sensor_msgs/PointCloud2`
- `/rgbd_2/points` : `gz.msgs.PointCloudPacked` → `sensor_msgs/PointCloud2`

## 节点说明

### TswGzSimNode

管道沿 X 轴匀速移动控制节点。

**参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `set_tube_length` | 1.0 | 管道行程长度 (m)，从 -length/2 移到 +length/2 |
| `speed` | 0.1 | 移动速度 (m/s) |
| `control_timer_ms` | 10 | 控制周期 (ms) |

**订阅**：`tube_cmd` (`std_msgs/String`) — 接收键盘指令

### KeyboardNode

终端键盘输入节点，发布控制指令到 `tube_cmd`。

| 按键 | 指令 |
|------|------|
| `f` | 正向移动 |
| `b` | 反向移动 |
| `s` | 停止 |
| `q` | 退出 |

## 运行

```bash
# 构建
colcon build --packages-select tsm_gz_sim
source install/setup.bash

# 启动仿真（Gazebo + RViz + Bridge + TF + 控制节点）
ros2 launch tsm_gz_sim bringup.launch.py

# 键盘控制（新终端）
ros2 run tsm_gz_sim keyboard_node
```
