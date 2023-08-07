import numpy as np
from scipy.spatial import cKDTree

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

def consistency_list(target_cloud, clouds):
    consistency = []

    for cloud in clouds:
        distance = are_point_clouds_same(target_cloud, cloud, threshold=0.1)
        consistency.append(distance)

    return consistency

if __name__ == "__main__":
    # 创建一个示例目标点云
    target_cloud = np.array([
        [0, 0, 0],
        [1, 1, 1],
        [2, 2, 2],
    ])

    # 创建多个示例点云区域
    clouds = [
        np.array([
            [0, 0, 0],
            [1, 1, 1],
            [2, 2, 2],
        ]),
        np.array([
            [0, 0, 0.5],
            [1, 1, 1.5],
            [2, 2, 2.5],
        ]),
        np.array([
            [0, 0, 0.1],
            [1, 1, 1.1],
            [2, 2, 2.1],
        ])
    ]

    # 判断目标点云与每个点云是否一致并输出结果列表
    consistency = consistency_list(target_cloud, clouds)
    print("Consistency list:", consistency)