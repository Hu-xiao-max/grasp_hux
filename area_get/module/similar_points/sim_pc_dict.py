import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree



def are_point_clouds_same(pc1, pc2):
        #本质就是判断所有点之间的欧式距离是否小于阈值，如果都小于阈值认为两个点云是相同的
        if len(pc1) != len(pc2):
            return False

        tree1 = cKDTree(pc1)
        tree2 = cKDTree(pc2)

        # 计算点云1中的每个点到点云2的最近距离,所以哪怕是两个乱序的但是相同的点云输出结果也是相同的
        dist1, _ = tree1.query(pc2, k=1)#欧式距离
        # # 计算点云2中的每个点到点云1的最近距离
        # dist2, _ = tree2.query(pc1, k=1)


        # 如果所有距离都小于阈值，则认为两个点云相同
        # return np.all(dist1 <= threshold) and np.all(dist2 <= threshold)
        return np.all(dist1 <= 0.1)#设置阈值 

def find_similar_cloud_indices(clouds):
    n = len(clouds)
    visited = [False] * n
    similar_cloud_indices = {}

    for i in range(n):
        if visited[i]:
            continue

        group = [i]
        visited[i] = True

        for j in range(i + 1, n):
            distance = are_point_clouds_same(clouds[i], clouds[j])

            # 如果距离小于阈值，则认为这两个点云是相同的
            if distance :
                group.append(j)
                visited[j] = True

        similar_cloud_indices[i] = group

    return similar_cloud_indices

if __name__ == "__main__":
    # 创建多个示例点云区域
    clouds = [
        np.array([
            [0, 0, 0],
            [1, 1, 1],
            [2, 2, 2],
        ]),
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
            [0, 0, 0.5],
            [1, 1, 1.5],
            [2, 2, 2.5],
        ])
    ]


    # 找出相同点云的索引
    similar_cloud_indices = find_similar_cloud_indices(clouds)
    print("Similar cloud indices:", similar_cloud_indices)

 