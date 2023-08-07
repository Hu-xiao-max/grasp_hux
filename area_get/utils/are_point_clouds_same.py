'''
在这个示例中，我们首先生成了两个包含4个点的点云。
然后，我们定义了一个名为are_point_clouds_same的函数，它接受两个点云和一个距离阈值作为参数。
该函数首先检查两个点云的点数是否相同，然后使用cKDTree构建点云的KD树，以加速最近邻搜索。最后，
我们计算两个点云之间的双向最近距离，并检查它们是否都小于给定阈值。如果是，则认为两个点云相同。
'''
import numpy as np
from scipy.spatial import cKDTree

# 生成两个点云示例
point_cloud1 = np.array([[0, 0, 0], [1, 1, 1], [2, 2, 2], [3, 3, 3]])
point_cloud2 = np.array([[0, 0, 0], [2, 2, 2], [1, 1, 1], [3, 3, 3.01],[3, 3.02, 3.01]])

# 判断点云是否相同的函数
def are_point_clouds_same(pc1, pc2, threshold=0.1):
    if len(pc1) != len(pc2):
        return False

    tree1 = cKDTree(pc1)
    tree2 = cKDTree(pc2)

    # 计算点云1中的每个点到点云2的最近距离
    dist1, _ = tree1.query(pc2, k=1)
    # 计算点云2中的每个点到点云1的最近距离
    dist2, _ = tree2.query(pc1, k=1)

    # 如果所有距离都小于阈值，则认为两个点云相同
    return np.all(dist1 <= threshold) and np.all(dist2 <= threshold)

# 使用示例点云测试函数
result = are_point_clouds_same(point_cloud1, point_cloud2)
print("Are point clouds same?", result)