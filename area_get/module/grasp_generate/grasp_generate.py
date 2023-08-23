import numpy as np
import time
import open3d as o3d
from collections import defaultdict
from scipy.spatial import cKDTree
import sys
import time
import datetime
import os

class grasp_config():
    def __init__(self, filepath):
            self.filepath=filepath
            self.point_clouds=self.read_obj_vertices()
            #物体点云downsample按照0.01m downsample
            self.point_clouds=self.downsample_points(self.point_clouds,voxel_size=0.01)
            #print(self.point_clouds.shape)
            # 物体的点法向量
            self.normals,self.pcd=self.generate_normals()
            #生成1296个抓取向量
            self.grasp_vectors=self.generate_unit_sphere_points(1296)
            # 生成抓取坐标系
            self.grasp_x,self.grasp_y=self.rotate_points_around(self.grasp_vectors)
            #夹爪尺寸参照GPD的Antipod抓取
            self.gripper_size = np.array([0.03, 0.08, 0.05])

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
    def downsample_points(self,points,voxel_size):
        """
        使用 Voxel Grid Filter 对点云进行降采样。

        参数：
            points (numpy.ndarray): 点云数据，shape 为 (N, 3)。
            voxel_size (float): 体素大小（立方体边长）。

        返回：
            numpy.ndarray: 降采样后的点云，shape 为 (M, 3)。
        """
        #voxel_size=0.005#按照1mm的粒度降采样，downsample的粒度
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)
        downsampled_point_cloud = point_cloud.voxel_down_sample(voxel_size)
        return np.asarray(downsampled_point_cloud.points)
    
    def generate_normals(self):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.point_clouds)
        k=10#k控制的是搜索的neighbour
        o3d.geometry.PointCloud.estimate_normals(pcd, search_param=o3d.geometry.KDTreeSearchParamKNN(k))
        normals = np.asarray(pcd.normals)
        return normals,pcd

    

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
    
    def rotate_points_around(self,points):
    # 生成抓取向量的x轴、y轴
        angle_radians = np.deg2rad(90)
        rotation_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(angle_radians), -np.sin(angle_radians)],
            [0, np.sin(angle_radians), np.cos(angle_radians)]
        ])
        rotated_points_x = np.dot(points, rotation_matrix.T)

        angle_radians = np.deg2rad(90)
        rotation_matrix = np.array([
            [np.cos(angle_radians), 0, np.sin(angle_radians)],
            [0, 1, 0],
            [-np.sin(angle_radians), 0, np.cos(angle_radians)]
        ])
        rotated_points_y = np.dot(points, rotation_matrix.T)

        return rotated_points_x,rotated_points_y

    
    def compute_contact(self,gripper_direction):
        # 计算夹爪方向与点云法向量之间的夹角
        angles = np.arccos(np.dot(self.normals, gripper_direction))
        # 定义一个阈值来选择可接触的点（这里假设夹角小于 45 度的点可以接触）
        # 提取可接触的点
        contact_points = np.asarray(self.pcd.points)[np.logical_and(np.radians(160) < angles, angles < np.radians(180))]
        
        # 将可接触的点保存到新的点云对象并可视化
        # contact_points_pcd = o3d.geometry.PointCloud()
        # contact_points_pcd.points = o3d.utility.Vector3dVector(contact_points)
        # o3d.visualization.draw_geometries([contact_points_pcd])
        return contact_points
    

    def is_point_colliding_with_point_cloud(self,point, point_cloud):
        threshold_distance = 0.001  # Set a threshold distance for collision detection
        kdtree = cKDTree(point_cloud)
        nearest_distance, nearest_index = kdtree.query(point)
        if nearest_distance <= threshold_distance:
            return True#返回True代表发生碰撞了
        else:
            return False#代表没发生碰撞

    def is_gripper_colliding_with_point_cloud(self, gripper_grasp_point, gripper_grasp_vector, point_cloud):
        # Compute the positions of the gripper fingertips in the object coordinate system
        gripperwidth_size=self.gripper_size[1]
        half_gripperwidth_size = gripperwidth_size / 2
        fingertip1_position = gripper_grasp_point + half_gripperwidth_size * gripper_grasp_vector
        fingertip2_position = gripper_grasp_point - half_gripperwidth_size * gripper_grasp_vector

        # Check if the fingertips are colliding with the point cloud
        fingertip1_colliding = self.is_point_colliding_with_point_cloud(fingertip1_position, point_cloud)
        fingertip2_colliding = self.is_point_colliding_with_point_cloud(fingertip2_position, point_cloud)
        return fingertip1_colliding or fingertip2_colliding

    


if __name__=='__main__':
    start=time.time()
    filepath='./object/nontextured.obj'
    graspconfig=grasp_config(filepath)
    count=0
    for contact_z, contact_x, contact_y in zip(graspconfig.grasp_vectors, graspconfig.grasp_x, graspconfig.grasp_y):
    # 遍历1296个抓取向量生成抓取点
        contact_points=graspconfig.compute_contact(contact_z)
        if contact_points.shape[0]>2:
            for contact_point in contact_points:
                if not graspconfig.is_gripper_colliding_with_point_cloud(contact_point, contact_y, graspconfig.point_clouds):
                    print('contact_point={},grasp_z={}'.format(contact_point,contact_z))
                    count += 1
                    break
    end=time.time()
    print(end-start)
    print(count)
    print(graspconfig.point_clouds.shape)


     
    

        
                