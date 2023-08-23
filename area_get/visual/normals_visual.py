import open3d as o3d
import numpy as np

class PointCloudProcessor:
    def __init__(self, point_clouds):
        self.point_clouds = point_clouds

    def generate_normals(self):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.point_clouds)
        k = 10  # k控制的是搜索的neighbour
        o3d.geometry.PointCloud.estimate_normals(pcd, search_param=o3d.geometry.KDTreeSearchParamKNN(k))
        normals = np.asarray(pcd.normals)

        # 计算点云的中心点
        center = np.mean(self.point_clouds, axis=0)

        # 判断法向量是否指向物体外部，如果不是则翻转
        for i in range(normals.shape[0]):
            vector_to_center = center - self.point_clouds[i]
            dot_product = np.dot(normals[i], vector_to_center)
            if dot_product > 0:#大于0代表同向
                normals[i] = -normals[i]

        # 更新点云的法向量
        #pcd.normals = o3d.utility.Vector3dVector(normals)

        return pcd, normals

# 创建一个随机点云
def create_random_point_cloud(num_points=1000):
    points = np.random.rand(num_points, 3)
    return points

# 可视化点云和法向量
def visualize_point_cloud_with_normals(pcd):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.get_render_option().point_show_normal = True
    vis.run()
    vis.destroy_window()

def read_obj_vertices(objfilepath):
        vertices = []

        with open(objfilepath, 'r') as file:
            for line in file:
                if line.startswith('v '):
                    vertex = line.strip().split(' ')[1:]
                    vertex = [float(coord) for coord in vertex]
                    vertices.append(vertex)
            vertices=np.array(vertices)

        return vertices

# 主程序
if __name__ == "__main__":
    objfilepath ='./area_get/object/nontextured.obj'
    point_cloud = read_obj_vertices(objfilepath)
    processor = PointCloudProcessor(point_cloud)
    pcd, normals = processor.generate_normals()
    visualize_point_cloud_with_normals(pcd)