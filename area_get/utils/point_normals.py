import numpy as np
'''
在这个示例中，我们定义了一个函数 compute_normals，
它接受一个点云数组和一个参数 k（表示最近邻点的数量）。
对于点云中的每个点，我们计算了它与其他点的距离，并找到 k 个最近邻点。
然后，我们计算了最近邻点的协方差矩阵，并对其进行了奇异值分解（SVD）。
最小奇异值对应的右奇异向量即为法向量。我们将所有点的法向量收集到一个数组中，并返回。

'''
def compute_normals(points, k=5):
    normals = []

    for i, point in enumerate(points):
        # 计算当前点与其他点的距离
        distances = np.linalg.norm(points - point, axis=1)

        # 找到 k 个最近邻点
        nearest_indices = np.argpartition(distances, k)[:k]
        nearest_points = points[nearest_indices]

        # 计算最近邻点的协方差矩阵
        covariance_matrix = np.cov(nearest_points.T)

        # 对协方差矩阵进行奇异值分解（SVD）
        _, _, vh = np.linalg.svd(covariance_matrix)

        # 最小奇异值对应的右奇异向量即为法向量
        normal = vh[-1]

        normals.append(normal)

    return np.array(normals)

# 示例点云数据（请替换为实际的点云数据）
points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 1]])

# 计算点云法向量
normals = compute_normals(points, k=3)

print("法向量：")
print(normals)