import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
'''
函数计算相同点云的数量。这个函数遍历每两个点云，计算它们之间的距离，
并根据距离阈值判断它们是否相同。如果距离小于阈值，则认为这两个点云是相同的，并将计数器加一。
'''

# 判断点云是否相同的函数
def are_point_clouds_same(pc1, pc2, threshold):
    #本质就是判断所有点之间的欧式距离是否小于阈值，如果都小于阈值认为两个点云是相同的
    if len(pc1) != len(pc2):
        return False

    tree1 = cKDTree(pc1)
    tree2 = cKDTree(pc2)

    # 计算点云1中的每个点到点云2的最近距离,所以哪怕是两个乱序的但是相同的点云输出结果也是相同的
    dist1, _ = tree1.query(pc2, k=1)#欧式距离
    # # 计算点云2中的每个点到点云1的最近距离
    # dist2, _ = tree2.query(pc1, k=1)
    print(dist1)


    # 如果所有距离都小于阈值，则认为两个点云相同
    # return np.all(dist1 <= threshold) and np.all(dist2 <= threshold)
    return np.all(dist1 <= threshold) 




def count_similar_clouds(clouds):
    n = len(clouds)
    count = 0

    for i in range(n):
        for j in range(i+1, n):
            #distance = nearest_neighbor_distance(clouds[i], clouds[j])
            print(i,j,'--------------------------')
            distance =are_point_clouds_same(clouds[i], clouds[j], threshold=0.4)

            # 如果距离小于阈值，则认为这两个点云是相同的
            
            if distance :
                count += 1

    return count

def calculate_repetition_rate(clouds):
    n = len(clouds)
    total_combinations = n * (n - 1) // 2
    similar_clouds_count = count_similar_clouds(clouds)

    return similar_clouds_count / total_combinations

if __name__ == "__main__":
    # 创建多个示例点云区域
    clouds = [
        np.array([
            [0, 0, 0],
            [1, 1, 1],
            [2, 2, 2],
        ]),
        np.array([
            [2, 3, 3],
            [1, 1, 1],
            [0, 0, 0],
        ]),
        np.array([
            [0, 0, 0.5],
            [1, 1, 1.5],
            [2, 2, 2.5],
        ]),
        np.array([
            [0, 0, 0.1],
            [1, 1, 1.1],
            [2, 2, 2.5],
        ])
    ]


    # 计算点云的重复率
    repetition_rate = calculate_repetition_rate(clouds)
    print("Repetition rate:", repetition_rate)