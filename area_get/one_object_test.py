import numpy as np
import time
import open3d as o3d
from collections import defaultdict
from scipy.spatial import cKDTree
import sys
import time
import datetime
import os


class Area_get():
    def __init__(self, filepath):
        self.filepath=filepath
        self.point_clouds=self.read_obj_vertices()
        #物体点云downsample按照0.01m downsample
        self.point_clouds=self.downsample_points(self.point_clouds)
        #生成1296个抓取向量
        self.grasp_vectors=self.generate_unit_sphere_points(1296)
        #夹爪尺寸参照GPD的Antipod抓取
        self.gripper_size = np.array([0.03, 0.05, 0.08])

        self.grasp_interaction_area=[]
        self.list_contact_points_shape=[]

    def read_obj_vertices(self):
        vertices = []

        with open(self.filepath, 'r') as file:
            for line in file:
                if line.startswith('v '):
                    vertex = line.strip().split(' ')[1:]
                    vertex = [float(coord) for coord in vertex]
                    vertices.append(vertex)
            vertices=np.array(vertices)

        return vertices
    
    def downsample_points(self,points):
        """
        使用 Voxel Grid Filter 对点云进行降采样。

        参数：
            points (numpy.ndarray): 点云数据，shape 为 (N, 3)。
            voxel_size (float): 体素大小（立方体边长）。

        返回：
            numpy.ndarray: 降采样后的点云，shape 为 (M, 3)。
        """
        voxel_size=0.005#按照1mm的粒度降采样，downsample的粒度
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)
        downsampled_point_cloud = point_cloud.voxel_down_sample(voxel_size)
        return np.asarray(downsampled_point_cloud.points)
    
    def compute_contact(self,gripper_direction):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.point_clouds)
        k=10#k控制的是搜索的neighbour
        smoothed_pcd = o3d.geometry.PointCloud.estimate_normals(pcd, search_param=o3d.geometry.KDTreeSearchParamKNN(k))
        # 计算点云的法向量
        smoothed_pcd=pcd
        normals = np.asarray(smoothed_pcd.normals)

        # 计算夹爪方向与点云法向量之间的夹角
        angles = np.arccos(np.dot(normals, gripper_direction))
        # 定义一个阈值来选择可接触的点（这里假设夹角小于 45 度的点可以接触）
        # 提取可接触的点
        contact_points = np.asarray(smoothed_pcd.points)[angles<np.radians(45)]
        return contact_points
    
    def generate_unit_sphere_points(self,num_points):
        #生成抓取向量
        indices = np.arange(0, num_points, dtype=float) + 0.5
        phi = np.arccos(1 - 2 * indices / num_points)
        theta = np.pi * (1 + 5**0.5) * indices

        x = np.cos(theta) * np.sin(phi)
        y = np.sin(theta) * np.sin(phi)
        z = np.cos(phi)
        points = np.stack((x, y, z), axis=-1)

        return points
    

    def transform_to_gripper_space(self,points, contact_point, contact_vector):
        # 计算夹爪的局部坐标系基向量
        gripper_z_axis = contact_vector / np.linalg.norm(contact_vector)
        
        # 随机选择一个与接触向量垂直的向量作为夹爪的 X 轴
        random_vector = np.random.randn(3)
        gripper_x_axis = np.cross(random_vector, gripper_z_axis)
        
        # 如果 X 轴和 Z 轴平行，重新选择一个随机向量
        while np.linalg.norm(gripper_x_axis) < 1e-6:
            random_vector = np.random.randn(3)
            gripper_x_axis = np.cross(random_vector, gripper_z_axis)
            
        #print(gripper_x_axis)
        
        gripper_x_axis /= np.linalg.norm(gripper_x_axis)
        gripper_y_axis = np.cross(gripper_z_axis, gripper_x_axis)

        # 计算点云到夹爪局部坐标系的变换矩阵
        rotation_matrix = np.array([gripper_x_axis, gripper_y_axis, gripper_z_axis]).T
        translation_vector = -np.dot(rotation_matrix, contact_point)

        # 将点云变换到夹爪局部坐标系
        return np.dot(points + translation_vector, rotation_matrix)

    def points_in_gripper_space(self, contact_point, contact_vector):
        points=self.point_clouds
        gripper_size=self.gripper_size
        # 将点云变换到夹爪局部坐标系
        points_transformed = self.transform_to_gripper_space(points, contact_point, contact_vector)

        # 计算夹爪长方体的 AABB
        gripper_aabb_max = gripper_size / 2
        gripper_aabb_min = -gripper_size / 2
        gripper_aabb_max[2]=gripper_aabb_max[2]-gripper_aabb_min[2]
        gripper_aabb_min[2]=0
        #更新为夹爪坐标系在夹爪中心
        #print(gripper_aabb_max,gripper_aabb_min)

        # 提取夹爪长方体空间内的点云
        mask = np.all((points_transformed >= gripper_aabb_min) & (points_transformed <= gripper_aabb_max), axis=1)
        return points[mask]
    


    def are_point_clouds_same(self,pc1, pc2):
        #本质就是判断所有点之间的欧式距离是否小于阈值，如果都小于阈值认为两个点云是相同的
        if len(pc1) != len(pc2):
            return False

        tree1 = cKDTree(pc1)
        tree2 = cKDTree(pc2)

        # 计算点云1中的每个点到点云2的最近距离,所以哪怕是两个乱序的但是相同的点云输出结果也是相同的
        dist1, _ = tree1.query(pc2, k=1)#欧式距离
        # # 计算点云2中的每个点到点云1的最近距离
        # dist2, _ = tree2.query(pc1, k=1)
        #print(dist1)


        # 如果所有距离都小于阈值，则认为两个点云相同
        # return np.all(dist1 <= threshold) and np.all(dist2 <= threshold)
        return np.all(dist1 <= 0.1)#设置阈值 
        
    def find_similar_cloud_indices(self,clouds):
        n = len(clouds)
        visited = [False] * n
        similar_cloud_indices = {}

        for i in range(n):
            if visited[i]:
                continue

            group = [i]
            visited[i] = True

            for j in range(i + 1, n):
                distance = self.are_point_clouds_same(clouds[i], clouds[j])

                # 如果距离小于阈值，则认为这两个点云是相同的
                if distance :
                    group.append(j)
                    visited[j] = True

            similar_cloud_indices[i] = group

        return similar_cloud_indices
    










    

if __name__ == '__main__':



    strat=time.time()
    # 获取当前时间
    now = datetime.datetime.now()

    if not os.path.exists('./area_get/output/'+now.strftime('%Y-%m-%d')):
        os.makedirs('./area_get/output/'+now.strftime('%Y-%m-%d'))
    f=open('./area_get/output/'+now.strftime('%Y-%m-%d')+'/'+now.strftime('%H-%M-%S') +".txt","w")
    sys.stdout=f

    filepath='./area_get/object/nontextured.obj'
    area_get=Area_get(filepath)
    #先是选择一个抓取向量然后基于抓取向量生成抓取点抓再根据抓取点提取夹爪点云
    for contact_vector in area_get.grasp_vectors:#遍历1296个抓取向量生成抓取点
    # gripper_direction = np.array([0, 0, 1])

        
        
        # print('contac_vector-------------------------------------------',contact_vector)    
        
        
        
        contact_points=area_get.compute_contact(contact_vector)
        #print(contact_points.shape)
        for contact_point in contact_points:#遍历抓取点生成夹爪点云
            points_in_gripper = area_get.points_in_gripper_space( contact_point, contact_vector)
            #print("Points in gripper space:", points_in_gripper.shape)#一个抓取点和抓取向量下的夹爪点云
            if points_in_gripper.size != 0:

                # ！！！！！！！可以在这里储存抓取向量和抓取点对应的config！！！
                area_get.list_contact_points_shape.append(points_in_gripper.shape)
                area_get.grasp_interaction_area.append(points_in_gripper)

                
                # print(contact_point)


            else:
                continue
    #print(area_get.list_contact_points_shape)
    #print(area_get.grasp_interaction_area)
    

    # 创建一个字典，键为列表中的元素，值为一个包含该元素索引的列表
    index_dict = defaultdict(list)

    # 遍历列表，将元素的索引添加到相应的键值列表中
    for index, element in enumerate(area_get.list_contact_points_shape):
        index_dict[element].append(index)

    # 打印结果
    for element, index_list in index_dict.items():
        #print(element, index_list)
        #print(area_get.grasp_interaction_area[index_list[0]])
        clouds=[]
        for clouds_index in index_list:
            clouds.append(area_get.grasp_interaction_area[clouds_index])
        similar_cloud_indices = area_get.find_similar_cloud_indices(clouds)
        print("Similar cloud indices:",element, similar_cloud_indices)


        
    end=time.time()
    print(end-strat)


