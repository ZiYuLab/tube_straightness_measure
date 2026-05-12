# CLAUDE.md

本文件用于说明 Claude Code 在本仓库中的工作方式、项目结构和关键运行约定。

## 项目概述

本项目是一个基于 ROS 2 + Gazebo Harmonic 的大直径钢管直度测量系统，主要由两个包组成：

- `tsm`：测量与算法核心
- `tsm_gz_sim`：Gazebo 仿真、TF、桥接与交互控制

当前代码重点在“仿真中采集双目/双 RGBD 点云，并逐帧估计钢管中心线相关量”。

## 构建与运行

在工作区根目录执行：

```bash
colcon build --packages-select tsm_gz_sim tsm
source install/setup.bash
```

启动仿真：

```bash
ros2 launch tsm_gz_sim bringup.launch.py
```

可选启动参数：

```bash
# 显式指定直管（默认）
ros2 launch tsm_gz_sim bringup.launch.py tube_type:=straight

# 指定弯管模型
ros2 launch tsm_gz_sim bringup.launch.py tube_type:=bend

# 给两个相机增加共同的 y 方向安装偏差
ros2 launch tsm_gz_sim bringup.launch.py camera_y_bias:=0.05
```

当前仓库没有完善的自动化测试；修改后优先通过 `colcon build` 验证编译。

## 代码结构

### `tsm_gz_sim`：仿真与控制

主要文件：

- `launch/bringup.launch.py`
  - 启动 Gazebo 世界 `measure_world.sdf`
  - 生成钢管模型（直管或弯管）
  - 启动 `ros_gz_bridge`
  - 发布两个 RGBD 相机的静态 TF
  - 启动 RViz2
  - 启动 `tsm_gz_sim_node`
- `config/sim_config.yaml`
  - 仿真节点参数
- `config/topic_bridge_config.yaml`
  - Gazebo 与 ROS 话题桥配置
- `src/tsm_gz_sim_node.cpp`
  - 管子运动、扰动注入、`world -> tube` TF 广播
- `src/keyboard_node.cpp`
  - 键盘控制节点，向 `tube_cmd` 发布 `f / b / s`
- `sdf/straight_tube/straight_tube.sdf`
  - 直管模型
- `sdf/bend_tube/bend_tube.sdf`
  - 弯管模型
- `rviz/rviz.rviz`
  - RViz 配置

#### `TsmGzSimNode`

`tsm_gz_sim_node` 的职责：

- 通过 `gz transport` 的 `/world/measure_world/set_pose` 服务驱动钢管模型运动
- 按定时器推进钢管沿 x 方向前进/后退
- 发布 `world -> tube` TF
- 向模型姿态中注入仿真扰动：
  - y/z 平移振动
  - 绕 z 轴的静态倾角与角振动

订阅：

- `tube_cmd`：字符串命令，`f` 前进，`b` 后退，`s` 停止

#### `KeyboardNode`

`keyboard_node` 在终端原始输入模式下读取按键：

- `f`：前进
- `b`：后退
- `s`：停止
- `q`：退出键盘节点

### `tsm`：测量与算法核心

主要文件：

- `src/tsm_node.cpp`
  - 主流程：点云同步、融合、裁剪、PCA、分段、统计、日志
- `src/fit_each_segment.cpp`
  - 单段圆拟合：RANSAC 初值 + Ceres 优化
- `src/integrator.cpp`
  - 对二阶差分做两次积分，恢复中心线
- `src/wls.cpp`
  - 结合曲率项和绝对位置项的加权最小二乘重建
- `config/tsm_config.yaml`
  - 测量相关参数
- `include/tsm/tsm_node.hpp`
  - `TsmNode` 声明

#### `TsmNode`

`tsm_node` 当前流程如下：

1. 订阅并同步两个点云：
   - `rgbd_1/points`
   - `rgbd_2/points`
2. 查询 TF：
   - 静态相机到世界坐标变换
   - 动态 `world -> tube` 变换
3. 将两个点云变换到 `world` 坐标系并合并
4. 按 `valid_pc_area` 参数做空间裁剪
5. 发布融合点云用于调试
6. 对点云做体素滤波
7. 使用 PCA 估计钢管轴向
8. 将点云旋转到“轴向对齐 x 轴”的坐标系
9. 沿 x 方向切成三段：前、中、后
10. 分别对三段截面做圆拟合，得到三个圆心
11. 用三个圆心估计二阶差分（曲率近似）
12. 把结果变换到 `tube` 坐标系，并按 `bin_length` 离散累计到 bin 中
13. 输出实时统计日志

当前节点会发布以下调试结果：

- 融合点云
- 三段切分后的点云
- 拟合过程中使用的调试图像接口已预留
- 中心线 Marker 发布器已创建，但是否完整使用应以当前代码为准

## 当前算法实现要点

### 1. 单段截面圆拟合

在 `fit_each_segment.cpp` 中，单段点云会先投影到 y-z 平面，然后：

1. 用 RANSAC 从三点构圆得到较好初值
2. 用 Ceres 最小化点到圆的残差，优化圆心和半径
3. 得到该段的截面圆心 `(x, y, z)`

可选地，还可以通过 `measurement_noise_sigma` 给圆心注入高斯测量噪声，用于仿真误差分析。

### 2. 三段圆心求二阶差分

在 `tsm_node.cpp` 中，使用三段的圆心：

- `center1`
- `center2`
- `center3`

计算中段对应的二阶差分，用于近似中心线曲率信息。

### 3. 两种中心线恢复方法

当前代码中实现了两种后处理方法：

#### `integrator.cpp`

对每个 bin 的平均二阶差分：

- 第一次积分得到斜率
- 第二次积分得到挠度

这是纯积分恢复，形式简单，但对噪声较敏感。

#### `wls.cpp`

构建加权最小二乘系统，融合两类观测：

- 曲率项（由二阶差分给出）
- 绝对位置项（由中段圆心给出）

并加入平滑正则项 `lambda`。这是当前更完整的中心线恢复实现。

## 关键参数

### `tsm/config/tsm_config.yaml`

#### `cutting_fittting`

- `length_of_each_segment`：每段沿钢管轴向的长度
- `ransac_max_iterations`：RANSAC 最大迭代次数
- `ransac_distance_threshold`：RANSAC 内点距离阈值
- `min_points_in_segment`：单段最少点数
- `measurement_noise_sigma`：截面圆心测量噪声标准差

#### `integral_process`

- `bin_length`：沿钢管轴向分箱长度
- `w_kappa`：曲率项权重
- `w_abs`：绝对位置项权重
- `lambda`：平滑正则项权重

### `tsm_gz_sim/config/sim_config.yaml`

- `control_timer_ms`：仿真控制周期
- `tube_length`：钢管长度
- `speed`：钢管移动速度
- `set_pose_service`：Gazebo 设置位姿服务名
- `tube_model_name`：模型名称
- `tf_frame_id` / `tf_child_frame_id`：TF 父子坐标系
- `vib_amplitude` / `vib_frequency`：平移振动参数
- `tilt_angle`：静态倾角
- `tilt_vib_amplitude` / `tilt_vib_frequency`：角振动参数

## 坐标系与 TF 约定

当前启动脚本中会发布两个相机的静态 TF：

- `world -> rgbd_1/camera_link/rgbd_1`
- `world -> rgbd_2/camera_link/rgbd_2`

同时 `tsm_gz_sim_node` 会持续发布：

- `world -> tube`

`tsm_node` 的数据流依赖这些 TF：

- 先把点云统一到 `world`
- 再结合 PCA 对齐结果和 `world -> tube`，把二阶差分与绝对中心位置转换到 `tube` 坐标系中累计

如果测量结果异常，优先检查：

- 点云是否确实在 `world` 下成功融合
- `world -> tube` 是否随钢管运动正确更新
- PCA 主方向是否稳定
- `pos_tube_x` 与 bin 索引是否符合预期

## 调试建议

当前代码很依赖 RViz 和日志调试。排查问题时建议优先看：

1. 融合点云是否完整
2. 三段切分点云是否都落在钢管上
3. `Seg1 / Seg2 / Seg3` 点数是否足够
4. bin 数量、当前 bin 计数、覆盖长度是否合理
5. `world -> tube` TF 与 Gazebo 中钢管位姿是否一致

## 依赖与工具链

- C++20
- ROS 2
- Gazebo Harmonic
- PCL
- Eigen
- OpenCV
- Ceres Solver

如需安装 Ceres，可参考：

- `scripts/install_ceres.sh`

## 代码风格

使用仓库根目录下的 `.clang-format`：

- 2 空格缩进
- 80 列换行
- `BreakBeforeBraces: Custom`
- `PointerAlignment: Left`
- 不自动排序 include

格式化示例：

```bash
clang-format -i tsm/src/tsm_node.cpp
```

## 对 Claude Code 的工作要求

1. 优先基于当前代码判断行为，不要依赖过时文档。
2. 修改测量逻辑时，注意区分：
   - `world` 坐标系下的点云处理
   - PCA 对齐后的中间坐标系
   - `tube` 坐标系下的最终累计量
3. 如果修改 bin、曲率、中心线恢复相关逻辑，优先同时检查日志输出是否会误导调试。
4. 除非用户明确要求，否则不要创建额外的设计文档；优先更新现有代码与本文件。
5. 汇报问题时，优先给出可点击代码位置。