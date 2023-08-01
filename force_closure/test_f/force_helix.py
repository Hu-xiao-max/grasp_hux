import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

'''力螺旋（force helix）是一个描述力和力矩在三维空间中的关系的几何概念。'''

# 定义力 F 和力矩 M
F = np.array([1, 0, 0])
M = np.array([0, 1, 0])

# 计算@@力螺旋系数@@，力螺旋系数
force_helix_coefficient = np.cross(F, M)#向量叉乘

# 定义力螺旋的长度和采样点数
length = 10
num_points = 100

# 生成力螺旋上的点
t = np.linspace(0, length, num_points)
x = F[0] * t + force_helix_coefficient[0] * np.sin(t)
y = F[1] * t + force_helix_coefficient[1] * np.sin(t)
z = F[2] * t + force_helix_coefficient[2] * np.sin(t)

# 绘制力螺旋
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()