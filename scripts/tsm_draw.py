import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ================= 解决中文显示异常 =================
plt.rcParams["font.sans-serif"] = [
    "PingFang SC",  # macOS 默认
    "Arial Unicode MS",  # macOS / Windows 备用
    "WenQuanYi Micro Hei",  # Linux (Ubuntu) 常用
    "Noto Sans CJK SC",  # Linux 常用
    "Microsoft YaHei",  # Windows 微软雅黑
    "SimHei",  # Windows 黑体
    "sans-serif",
]
plt.rcParams["axes.unicode_minus"] = False

# ================= 数据文件与配置 =================
files = [
    {"path": "abs_track.csv", "name": "绝对坐标测量", "color": "#1f77b4"},
    {"path": "integrator_track.csv", "name": "本方法", "color": "#ff7f0e"},
    # {"path": "wls_track.csv", "name": "WLS 轨迹", "color": "#2ca02c"},
]

fig, ax = plt.subplots(figsize=(12, 6))

max_x_overall = 0

# 遍历绘制每一条测量曲线
for file_info in files:
    df = pd.read_csv(file_info["path"])
    x = df["u"]
    y = df["dist"]

    # 更新全局最大 X 跨度 (用于确定实际弦长)
    current_max_x = x.max()
    if current_max_x > max_x_overall:
        max_x_overall = current_max_x

    # 动态计算最大弓高（直线度）及其坐标
    max_idx = y.idxmax()
    max_x = x.iloc[max_idx]
    max_y = y.iloc[max_idx]

    label_text = f"{file_info['name']} (直线度: {max_y:.4f} m)"

    # 绘制主曲线
    ax.plot(x, y, label=label_text, color=file_info["color"], linewidth=2)

    # 重点标出最大弓高点及辅助线
    ax.plot(max_x, max_y, marker="*", markersize=12, color="red", zorder=5)
    ax.vlines(
        x=max_x,
        ymin=0,
        ymax=max_y,
        colors=file_info["color"],
        linestyles="dashed",
        alpha=0.7,
    )
    ax.annotate(
        f"最大: {max_y:.4f}",
        xy=(max_x, max_y),
        xytext=(0, 8),
        textcoords="offset points",
        ha="center",
        va="bottom",
        color=file_info["color"],
        fontsize=10,
        fontweight="bold",
    )

# ================= 理论参考轴线 (基于固定曲率，端点对齐) =================
# 1. 物理基准：10m长对应10度角 -> 确定固定的曲率半径 R
base_length = 10.0
base_angle_rad = np.radians(10.0)
R = base_length / base_angle_rad  # R 约为 57.296 m

# 2. 实际测量的弦长：使用数据的最大 X 跨度
chord_length = max_x_overall

# 3. 计算圆心到弦的垂直距离 d (利用直角三角形)
d = np.sqrt(R**2 - (chord_length / 2) ** 2)

# 4. 构建圆弧解析方程
# 将圆心设在 (chord_length/2, -d)，保证圆弧穿过 (0,0) 和 (chord_length,0)
x_ref = np.linspace(0, chord_length, 500)
y_ref = np.sqrt(R**2 - (x_ref - chord_length / 2) ** 2) - d

# 理论在此实际弦长下的最大弓高
ref_max_y = R - d

# 绘制理论参考线
ax.plot(
    x_ref,
    y_ref,
    color="black",
    linestyle="-.",
    linewidth=2,
    label=f"理论参考轴线 (固定基准: 10m/10°)\n此端点间理论弓高: {ref_max_y:.4f} m",
)

# ================= 图表排版与装饰 =================
ax.set_xlabel("距离 X (m)", fontsize=12)
ax.set_ylabel("测量距离 / 弓高 Y (m)", fontsize=12)
ax.set_title("钢管直度测量数据曲线对比", fontsize=15, pad=15)

ax.set_xlim(left=0)

ax.grid(True, linestyle="--", alpha=0.5)
ax.legend(loc="best", framealpha=0.9)

plt.tight_layout()

# 保存图像并显示
plt.savefig("straightness_plot_fixed_radius.png", dpi=300)
plt.show()
