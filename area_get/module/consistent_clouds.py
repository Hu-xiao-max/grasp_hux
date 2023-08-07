import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

def downsample_points(points, voxel_size):
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    downsampled_point_cloud = point_cloud.voxel_down_sample(voxel_size)
    return np.asarray(downsampled_point_cloud.points)

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

def find_consistent_clouds(clouds, voxel_size):
    n = len(clouds)
    consistent_clouds = []

    # 对每个点云进行降采样
    downsampled_clouds = [downsample_points(cloud, voxel_size) for cloud in clouds]

    # 计算每两个降采样后的点云之间的距离
    for i in range(n):
        for j in range(i+1, n):
            distance = are_point_clouds_same(downsampled_clouds[i], downsampled_clouds[j], threshold=0.1)

            # 如果距离小于阈值，则认为这两个降采样后的点云是一致的
            if distance:
                consistent_clouds.append((i, j))

    return consistent_clouds

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
            [2, 2, 2.5],
        ]),
        np.array([
            [0, 0, 0],
            [1, 1, 1],
            [2, 2, 2.5],
        ]),
        np.array([
            [0, 0, 0],
            [1, 1, 1],
            [2, 2, 2],
        ])
    ]

    # 设置体素大小和距离阈值
    voxel_size = 2.0


    # 找出降采样后一致的点云
    consistent_clouds = find_consistent_clouds(clouds, voxel_size)
    print("Consistent clouds:", consistent_clouds)