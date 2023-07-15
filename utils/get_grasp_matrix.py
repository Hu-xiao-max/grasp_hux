'''在这个示例中，我们首先定义了一个矩形物体的顶点，然后计算了物体的质心。
接着，我们计算了接触点相对于质心的向量，并根据摩擦系数计算了接触点之间的摩擦锥角度。
最后，我们计算了力闭合抓取的接触点力矩阵。

请注意，这个示例非常简化，仅用于演示如何使用 Python 计算力闭合抓取的接触点。
实际应用中，可能需要考虑更复杂的物体形状、机器人手的约束条件等因素。
此外，还可以使用专门的机器人学库，如 OpenRAVE、GraspIt! 等，来进行更复杂的力闭合抓取计算。'''

import numpy as np

# 定义物体（矩形）的顶点
object_vertices = np.array([[0, 0], [1, 0], [1, 1], [0, 1]])

# 计算物体的质心
centroid = np.mean(object_vertices, axis=0)#np.mean是求平均值

# 计算接触点相对于质心的向量
contact_vectors = object_vertices - centroid

# 计算接触点之间的摩擦锥角度（假设摩擦系数为 0.5）
friction_angle = np.arctan(0.5)

# 计算接触点的摩擦锥边界向量
friction_cone_vectors = np.zeros((4, 2, 2))

for i in range(4):
    angle = np.arctan2(contact_vectors[i, 1], contact_vectors[i, 0])
    friction_cone_vectors[i, 0] = np.array([np.cos(angle + friction_angle), np.sin(angle + friction_angle)])
    friction_cone_vectors[i, 1] = np.array([np.cos(angle - friction_angle), np.sin(angle - friction_angle)])

# 计算力闭合抓取的接触点
grasp_matrix = np.zeros((4, 2))

for i in range(4):
    grasp_matrix[i] = (friction_cone_vectors[i, 0] + friction_cone_vectors[i, 1]) / 2

print("接触点力矩阵：")
print(grasp_matrix)