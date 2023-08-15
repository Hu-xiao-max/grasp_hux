'''
将点云空间划分为大小相等的立方体（体素），然后用每个立方体内的点的平均值代替这些点。
体素的大小由阈值参数控制，较大的阈值将导致更粗糙的降采样结果。
'''

import numpy as np
import open3d as o3d

def downsample_points(points, voxel_size):
    """
    使用 Voxel Grid Filter 对点云进行降采样。

    参数：
        points (numpy.ndarray): 点云数据，shape 为 (N, 3)。
        voxel_size (float): 体素大小（立方体边长）。

    返回：
        numpy.ndarray: 降采样后的点云，shape 为 (M, 3)。
    """
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    downsampled_point_cloud = point_cloud.voxel_down_sample(voxel_size)
    return np.asarray(downsampled_point_cloud.points)




def read_obj_vertices(filepath):
        vertices = []

        with open(filepath, 'r') as file:
            for line in file:
                if line.startswith('v '):
                    vertex = line.strip().split(' ')[1:]
                    vertex = [float(coord) for coord in vertex]
                    vertices.append(vertex)
            vertices=np.array(vertices)

        return vertices

if __name__ == "__main__":
    # # 创建一个示例点云数据
    # points = np.array([
    #     [0, 0, 0],
    #     [1, 1, 1],
    #     [2, 2, 2],
    #     [3, 3, 3],
    #     [4, 4, 4],
    # ])
    filepath='./object/nontextured.obj'
    points=read_obj_vertices(filepath)

    # 设置体素大小（立方体边长）--》即为降采样粒度
    voxel_size = 0.001

    # 对点云进行降采样
    downsampled_points = downsample_points(points, voxel_size)
    print("Downsampled points:", downsampled_points)

    # 将可接触的点保存到新的点云对象并可视化
    contact_points_pcd = o3d.geometry.PointCloud()
    contact_points_pcd.points = o3d.utility.Vector3dVector(downsampled_points)
    o3d.visualization.draw_geometries([contact_points_pcd])
