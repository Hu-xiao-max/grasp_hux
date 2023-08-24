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
num_points = 360
unit_sphere_points = generate_unit_sphere_points(num_points)

# print(unit_sphere_points)
def rotate_points_around(points):
    angle_radians = np.deg2rad(90)
    rotation_matrix = np.array([
        [1, 0, 0],
        [0, np.cos(angle_radians), -np.sin(angle_radians)],
        [0, np.sin(angle_radians), np.cos(angle_radians)]
    ])
    rotated_points_x = np.dot(points, rotation_matrix.T)

    angle_radians = np.deg2rad(90)
    rotation_matrix = np.array([
        [np.cos(angle_radians), 0, np.sin(angle_radians)],
        [0, 1, 0],
        [-np.sin(angle_radians), 0, np.cos(angle_radians)]
    ])
    rotated_points_y = np.dot(points, rotation_matrix.T)

    return rotated_points_x,rotated_points_y

points = np.array([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
])
t1,t2=rotate_points_around(points)
print(t1,t2)


# 可视化点云
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(*zip(*unit_sphere_points), s=1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()

