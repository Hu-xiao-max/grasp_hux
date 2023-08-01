import numpy as np
import open3d as o3d

def save_points_as_pcd(points, output_file):
    """
    将一系列点保存为 PCD 文件。

    参数：
        points (numpy.ndarray): 点云数据，shape 为 (N, 3)。
        output_file (str): 输出 PCD 文件的路径。
    """
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(output_file, point_cloud)

if __name__ == "__main__":
    # 创建一个示例点云数据
    points = np.array([
        [0, 0, 0],
        [1, 1, 1],
        [2, 2, 2],
        [3, 3, 3],
        [4, 4, 4],
    ])

    # 保存点云数据为 PCD 文件
    output_file = "output.pcd"
    save_points_as_pcd(points, output_file)
    print(f"Saved points as PCD file: {output_file}")