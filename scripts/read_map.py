#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import os

# 代价地图文件路径
costmap_file = os.path.expanduser("~/.ros/costmap.csv")

# 读取 CSV 文件
costmap = np.loadtxt(costmap_file, delimiter=",")

# 放大窗口
plt.figure(figsize=(12, 10))  # 增大窗口大小

# 可视化代价地图（增强对比）
plt.imshow(costmap, cmap="gray", origin="lower", interpolation="nearest", vmin=0, vmax=255)

# 添加颜色条
plt.colorbar(label="Cost Value")

# 增加网格以辅助观察
plt.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)

# 添加标题和标签
plt.title("Enhanced Costmap Visualization", fontsize=14)
plt.xlabel("X (cells)", fontsize=12)
plt.ylabel("Y (cells)", fontsize=12)

# 显示图像
plt.show()
