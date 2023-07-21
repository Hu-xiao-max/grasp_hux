import numpy as np

# 定义物体（矩形）的顶点
object_vertices = np.array([[0, 0], [1, 0]])

# 计算物体的质心
centroid = np.mean(object_vertices, axis=0)

# 计算接触点相对于质心的向量
contact_vectors = object_vertices - centroid

print(contact_vectors)

# 计算接触点之间的摩擦锥角度（假设摩擦系数为 0.5）
friction_angle = np.arctan(0.5)

# 计算接触点的摩擦锥边界向量
friction_cone_vectors = np.zeros((2, 2, 2))
'''
friction_cone_vectors 数组中，数组的形状为 (2, 2, 2),
其中 2 表示接触点的数量，第一个 2 表示每个接触点有两个边界向量，
第二个 2 表示二维空间中的向量分量(x 和 y 坐标）
'''
for i in range(2):
    angle = np.arctan2(contact_vectors[i, 1], contact_vectors[i, 0])
    friction_cone_vectors[i, 0] = np.array([np.cos(angle + friction_angle), np.sin(angle + friction_angle)])
    friction_cone_vectors[i, 1] = np.array([np.cos(angle - friction_angle), np.sin(angle - friction_angle)])

# 计算力闭合抓取的接触点
grasp_matrix = np.zeros((2, 2))

for i in range(2):
    grasp_matrix[i] = (friction_cone_vectors[i, 0] + friction_cone_vectors[i, 1]) / 2

print("接触点力矩阵：")
print(grasp_matrix)

# 计算接触点力矩阵的行列式
determinant = np.linalg.det(grasp_matrix)

# 判断是否能抓取物体
can_grasp = determinant != 0#行列式不等于0代表接触力不是线性相关是稳定的

print("接触点力矩阵的行列式：", determinant)
print("是否能抓取物体：", can_grasp)