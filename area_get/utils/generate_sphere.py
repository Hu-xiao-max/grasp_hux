import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def generate_unit_sphere_points(num_points):
    indices = np.arange(0, num_points, dtype=float) + 0.5
    phi = np.arccos(1 - 2 * indices / num_points)
    theta = np.pi * (1 + 5**0.5) * indices

    x = np.cos(theta) * np.sin(phi)
    y = np.sin(theta) * np.sin(phi)
    z = np.cos(phi)
    points = np.stack((x, y, z), axis=-1)

    return points
#生成1296个抓取向量
num_points = 1296
unit_sphere_points = generate_unit_sphere_points(num_points)

print(unit_sphere_points)

# 可视化点云
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(*zip(*unit_sphere_points), s=1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()