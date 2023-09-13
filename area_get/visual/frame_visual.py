import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 球面上均匀分布的点的数量
num_points = 360

# 生成均匀分布的点
phi = np.random.uniform(0, np.pi, num_points)
theta = np.random.uniform(0, 2 * np.pi, num_points)

# 将球坐标转换为笛卡尔坐标
x = np.sin(phi) * np.cos(theta)
y = np.sin(phi) * np.sin(theta)
z = np.cos(phi)

# 创建3D图形
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制坐标系
scale = 0.1
for i in range(num_points):
    # x轴
    ax.quiver(x[i], y[i], z[i], scale, 0, 0, color='r', arrow_length_ratio=0.1)
    # y轴
    ax.quiver(x[i], y[i], z[i], 0, scale, 0, color='g', arrow_length_ratio=0.1)
    # z轴
    ax.quiver(x[i], y[i], z[i], 0, 0, scale, color='b', arrow_length_ratio=0.1)

# 设置坐标轴标签
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# 设置坐标轴范围
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

# 显示图形
plt.show()