# 大直径钢管直度测量系统算法设计

本文档描述当前 `tsm` 包中已实现的测量主流程、坐标定义、后处理方法，以及当前实现中的近似与限制。

## 1. 总体目标

系统目标是在仿真环境中使用两路 RGBD 点云，对钢管中心线进行逐帧采样，并在采样完成后计算：

- 绝对中心轨迹
- 基于二阶差分的积分重建轨迹
- 基于曲率项 + 绝对位置项的 WLS 重建轨迹
- 三种轨迹各自相对于参考线的弓高/直线度

当前主节点为：

- `tsm_node`

核心实现位于：

- `tsm/src/tsm_node.cpp`
- `tsm/src/fit_each_segment.cpp`
- `tsm/src/integrator.cpp`
- `tsm/src/wls.cpp`
- `tsm/src/post_process.cpp`

## 2. 输入与输出

### 2.1 输入

`tsm_node` 订阅两路同步点云：

- `rgbd_1/points`
- `rgbd_2/points`

节点还依赖以下 TF：

- 两个相机到 `world` 的静态 TF
- `world` 与 `tube` 的动态 TF

### 2.2 输出

当前实现会输出：

- 融合点云 `merged_points`
- 三段分割点云 `seg1_points` / `seg2_points` / `seg3_points`
- 采样完成后的轨迹 CSV：
  - `abs_track.csv`
  - `integrator_track.csv`
  - `wls_track.csv`

CSV 默认保存在 `tsm` 包 share 目录下的 `results/` 中。

## 3. 主流程

### 3.1 点云同步与融合

在 `tsm_node.cpp` 中，节点使用 `message_filters::Synchronizer` 对两路点云做近似时间同步。

收到同步点云后：

1. 查询 `world <- tube` 相关 TF
2. 查询两路相机到 `world` 的静态变换
3. 将两路点云都变换到 `world` 坐标系
4. 合并为单个点云 `merged`

### 3.2 有效区域裁剪

融合点云会根据参数 `valid_pc_area` 做空间裁剪，去掉钢管周围的背景点和明显离群点。

参数定义见：

- `tsm/config/tsm_config.yaml`

### 3.3 体素滤波与 PCA 主轴估计

裁剪后点云会先做体素滤波，以降低点密度不均对 PCA 的影响。

随后使用 PCA 估计当前可见钢管主轴方向：

- 若第一主方向与全局 x 轴夹角较小，则直接采用第一特征向量
- 否则退回第二特征向量

然后构造旋转，将该主轴对齐到 x 轴。

这一步的目的不是恢复精确的 `tube` 姿态，而是得到一个“当前可见钢管主轴已经拉平”的局部坐标系，便于后续切段和截面拟合。

## 4. 三段切分与圆拟合

### 4.1 沿 PCA 轴方向切三段

点云旋转到 PCA 对齐坐标系后：

- 统计当前点云沿 x 的最小值和最大值
- 按参数 `length_of_each_segment` 截取：
  - 左端一段
  - 中间一段
  - 右端一段

### 4.2 圆拟合

三段截面分别调用 `fitEachSegment()` 处理。

处理步骤如下：

1. 将该段点投影到 y-z 平面
2. 使用 RANSAC 三点构圆，得到较好初值
3. 使用 Ceres 对圆心与半径做非线性优化
4. 得到该段圆心 `center = (x, y, z)`

其中：

- `x` 是当前段中心在 PCA 对齐坐标系下的轴向位置
- `y/z` 是相对当前 PCA 主轴的横向偏移

## 5. 二阶差分与逐帧采样

### 5.1 二阶差分

设三段圆心分别为：

- `center1`
- `center2`
- `center3`

系统使用中间点的离散二阶差分近似局部曲率信息：

\[
ms = \frac{m_3 - 2m_2 + m_1}{d^2}
\]

其中：

- `m_i` 为各圆心在横向平面内的偏移量
- `d = center2.x() - center1.x()`

这里的 `ms` 本质上是在 PCA 对齐坐标系下估计得到的横向二阶变化量。

### 5.2 当前实现中的坐标约定

当前实现对中心位置和曲率采用了如下近似：

#### 5.2.1 x 方向

当前 `tube` 动态 TF 只可靠使用 x 平移，因此：

- 先将 `center2` 从 PCA 坐标系转回 `world`
- 再计算：

\[
pos\_tube\_x = center2\_world.x - tube\_world\_x
\]

也就是说，当前 `pos_tube_x` 表示：

- 中间截面中心在世界系的 x
- 减去当前钢管原点在世界系的 x

这给出了一个近似的钢管轴向采样位置。

#### 5.2.2 y/z 方向

当前实现中，绝对中心位置 `center2_tube` 的：

- `x` 使用上面的 `pos_tube_x`
- `y/z` 直接保留 PCA 对齐坐标系中的 `center2.y()` / `center2.z()`

这意味着当前绝对中心轨迹的定义是：

\[
(x_{tube\_approx}, y_{pca}, z_{pca})
\]

它并不是严格意义上的完整 `tube` 坐标系结果，而是：

- 用 TF 修正轴向位置
- 用 PCA 修正横向姿态

这种做法的优点是：

- 不需要知道钢管真实旋转
- 可以去掉整体小角度倾斜对横向偏移的直接污染

局限是：

- 若需要严格的三维刚体姿态恢复，则当前信息仍不足

#### 5.2.3 曲率量

当前代码中，二阶差分向量 `ms` 先被从 PCA 坐标系转回 `world`，随后直接作为 `ms_tube` 使用。

这意味着：

- 曲率幅值主要仍来自 PCA 对齐后的估计
- 但其 y/z 分量解释仍带有近似性

## 6. 分 bin 累计

逐帧采样会根据：

\[
bin\_idx = \lfloor pos\_tube\_x / bin\_length \rfloor
\]

将当前观测累计到两个 map 中：

- `diff_bins_`：存放二阶差分均值统计
- `abs_bins_`：存放绝对中心位置均值统计

每个 bin 内累计：

- `sum`
- `sum_sq`
- `count`

因此每个 bin 最终可恢复该位置上的平均观测值。

## 7. 三种轨迹恢复方法

### 7.1 Abs 方法

直接从 `abs_bins_` 中取每个 bin 的平均绝对中心位置，构造中心序列：

\[
(x_i, y_i, z_i)
\]

这是最直接的轨迹表示。

特点：

- 对刚体姿态和低频漂移最敏感
- 但最接近“原始采样中心位置”

### 7.2 Integrator 方法

在 `integrator.cpp` 中，对 `diff_bins_` 中的平均二阶差分做两次积分：

1. 曲率积分得到斜率
2. 斜率积分得到挠度

最终得到重建中心线。

特点：

- 直接利用曲率信息
- 可能出现积分漂移
- 对低频绝对位置约束较弱

### 7.3 WLS 方法

在 `wls.cpp` 中，系统构建一个加权最小二乘问题，联合使用：

- 曲率项 `w_kappa`
- 绝对位置项 `w_abs`
- 平滑正则项 `lambda`

并对绝对位置项先做线性去趋势。

特点：

- 相比纯积分更稳定
- 能融合绝对位置信息
- 会受到权重设置的显著影响

当前默认参数见：

- `tsm/config/tsm_config.yaml`

## 8. 弓高 / 直线度后处理

采样结束后，`postProcess()` 会生成三种方法的轨迹，并分别计算相对于各自参考线的偏离。

### 8.1 当前参考线定义

当前每条轨迹都使用“自己的首尾点连线”作为参考线。

也就是说：

- `abs` 用 `abs` 轨迹自己的首尾点
- `integrator` 用 `integrator` 轨迹自己的首尾点
- `wls` 用 `wls` 轨迹自己的首尾点

### 8.2 当前偏离计算

对于轨迹上的每个点：

1. 计算其到参考线的垂直向量 `perp`
2. 计算真实 3D 距离 `dist = ||perp||`
3. 保存：
   - `x`：沿参考线方向的位置
   - `signed_y`：带符号偏离
   - `distance`：真实距离

当前直线度/弓高输出值使用：

\[
\max(dist)
\]

也就是该轨迹相对自身首尾连线的最大偏离距离。

### 8.3 当前结果解释

由于每条轨迹都使用自己的首尾点作参考线，因此：

- 更适合做三种方法之间的相对比较
- 不一定等价于理论模型相对“理想端点连线”的弓高

如果理论弓高是基于理想几何端点定义的，那么当前实现的结果通常会偏小一些。

## 9. 参数说明

### `cutting_fittting`

- `length_of_each_segment`：每段长度
- `ransac_max_iterations`：RANSAC 最大迭代次数
- `ransac_distance_threshold`：RANSAC 内点阈值
- `min_points_in_segment`：单段最少点数
- `measurement_noise_sigma`：截面圆心测量噪声

### `integral_process`

- `bin_length`：沿钢管轴向的采样分箱长度
- `w_kappa`：曲率项权重
- `w_abs`：绝对位置项权重
- `lambda`：平滑正则项权重

## 10. 当前实现的主要近似与限制

1. `tube` 动态 TF 当前只可靠使用 x 平移。
2. 绝对中心轨迹使用的是：
   - `x` 的 tube 近似位置
   - `y/z` 的 PCA 横向偏移
3. 曲率向量的坐标解释仍是近似的。
4. 后处理参考线使用各自首尾点连线，因此更适合方法间比较，不完全等价于理论弓高定义。
5. WLS 中的去趋势会进一步削弱低频整体倾斜分量。

## 11. 当前推荐理解方式

如果当前目标是比较三种方案：

- `abs`
- `integrator`
- `wls`

则当前流程是合理的，因为三者共享同一套前端采样流程，只在后端重建方式不同。

如果目标是严格对齐理论模型的绝对弓高，则还需要进一步统一：

- 参考线定义
- 理论端点定义
- 完整的 `world -> tube` 姿态恢复方式

## 12. 启动方式

当前 `tsm` 包提供独立 launch 文件：

- `tsm/launch/bringup.launch.py`

启动命令：

```bash
ros2 launch tsm bringup.launch.py
```

默认会加载：

- `tsm/config/tsm_config.yaml`

也可自定义参数文件：

```bash
ros2 launch tsm bringup.launch.py params_file:=/your/path/custom.yaml
```
