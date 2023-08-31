import numpy as np
import time
import open3d as o3d
from collections import defaultdict
from scipy.spatial import cKDTree
import sys
import time
import datetime
import os
import math
import fcl
from fcl import CollisionObject, Box, DynamicAABBTreeCollisionManager, CollisionData, defaultCollisionCallback

class grasp_config():
    def __init__(self, filepath):
            self.filepath=filepath
            self.point_cloud=self.read_obj_vertices()
            #物体点云downsample按照0.01m downsample
            self.point_clouds=self.downsample_points(self.point_cloud,voxel_size=0.01)
            #print(self.point_clouds.shape)
            # 物体的点法向量
            self.normals = self.generate_normals()
            self.pcd=self.generate_pcd()
            #生成1296个抓取向量
            self.grasp_vectors=self.generate_unit_sphere_points(1296)
            # 生成抓取坐标系
            self.grasp_x,self.grasp_y=self.rotate_points_around(self.grasp_vectors)
            #夹爪尺寸参照GPD的Antipod抓取
            self.gripper_size = np.array([0.03, 0.08, 0.05])
            self.box1_depth=0.01

            # 作碰撞检测使用的夹爪模型
            self.box1, self.box2 , self.box3 = self.create_gripper()

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
    

    def generate_pcd(self):
        #qpc重定向，这样点云法向量都指向物体外部
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.point_clouds)
        k = 10  # k控制的是搜索的neighbour
        o3d.geometry.PointCloud.estimate_normals(pcd, search_param=o3d.geometry.KDTreeSearchParamKNN(k))
    
        return pcd
    
    def generate_normals(self):
        #qpc重定向，这样点云法向量都指向物体外部
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

        return normals


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

    
    def compute_contact(self, gripper_direction):
        # 计算夹爪方向与点云法向量之间的夹角
        gripper_direction = np.array(gripper_direction)
        gripper_direction_norm = np.linalg.norm(gripper_direction)
        normals_norm = np.linalg.norm(self.normals, axis=1)
        
        cos_angles = np.dot(self.normals, gripper_direction) / (normals_norm * gripper_direction_norm)
        angles = np.arccos(cos_angles)
        
        # 定义一个阈值来选择可接触的点（这里假设夹角小于 45 度的点可以接触）
        # 提取可接触的点
        contact_points = np.asarray(self.pcd.points)[np.logical_and(np.radians(135) < angles, angles < np.radians(180))]
        
        # 将可接触的点保存到新的点云对象并可视化
        # contact_points_pcd = o3d.geometry.PointCloud()
        # contact_points_pcd.points = o3d.utility.Vector3dVector(contact_points)
        # o3d.visualization.draw_geometries([contact_points_pcd])
        return contact_points
    

    def create_gripper(self,box1_width=0.03, box1_height=0.07, box1_depth=0.01, beam_height=0.02, box3_depth=0.03,
                            distance=0.08):
        box1 = o3d.geometry.TriangleMesh.create_box(width=box1_width, height=box1_depth, depth=box1_height)
        box1.translate((-beam_height / 2, -distance / 2 - box1_depth, -box3_depth / 2))
        box2 = o3d.geometry.TriangleMesh.create_box(width=box1_width, height=box1_depth, depth=box1_height)
        box2.translate((-beam_height / 2, distance / 2, -box3_depth / 2))

        box3 = o3d.geometry.TriangleMesh.create_box(width=box3_depth, height=distance, depth=beam_height)
        box3.translate(((box1_width - box3_depth) / 2 - beam_height / 2, -distance / 2, -box3_depth / 2))

        # origin_gripper = box1 + box2 + box3

        return box1, box2 , box3


    def create_gripper_model(self,position=(0, 0, 0),gripper_vector=(0, 0, 1),
                            contact_x=(1, 0, 0), contact_y=(0, 1, 0)):
        # 计算旋转
        rotation_matrix = np.column_stack((np.array(contact_x), np.array(contact_y), np.array(gripper_vector)))
        # 旋转并移动夹爪
        box1 = self.box1
        box1.rotate(rotation_matrix)
        box1.translate(position)

        box2 = self.box2
        box2.rotate(rotation_matrix)
        box2.translate(position)
        
        box3 = self.box3
        box3.rotate(rotation_matrix)
        box3.translate(position)
        return box1, box2, box3

    def create_triangle_mesh_from_point_cloud(self):
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(self.point_cloud)
        return mesh

    def check_collision(self, gripper_mesh):
        object_mesh = self.create_triangle_mesh_from_point_cloud()
        object_fcl = CollisionObject(Box(*object_mesh.get_max_bound() - object_mesh.get_min_bound()))
        object_fcl.setTranslation(object_mesh.get_center())
        gripper_fcl = CollisionObject(Box(*gripper_mesh.get_max_bound() - gripper_mesh.get_min_bound()))
        gripper_fcl.setTranslation(gripper_mesh.get_center())

        collision_manager = DynamicAABBTreeCollisionManager()
        collision_manager.registerObject(object_fcl)
        collision_manager.registerObject(gripper_fcl)
        collision_manager.setup()

        collision_data = CollisionData()
        collision_manager.collide(collision_data, defaultCollisionCallback)
        return collision_data.result.is_collision



if __name__=='__main__':
    now = datetime.datetime.now()
    if not os.path.exists('./area_get/output/'+now.strftime('%Y-%m-%d')):
        os.makedirs('./area_get/output/'+now.strftime('%Y-%m-%d'))
    f=open('./area_get/output/'+now.strftime('%Y-%m-%d')+'/'+now.strftime('%H-%M-%S') +".txt","w")
    sys.stdout=f

    start=time.time()
    filepath='./area_get/object/nontextured.obj'
    graspconfig = grasp_config(filepath)
    count=0

    f_alpha=45#摩擦系数
    for contact_z, contact_x, contact_y in zip(graspconfig.grasp_vectors, graspconfig.grasp_x, graspconfig.grasp_y):
    # 遍历1296个抓取向量生成抓取点
        contact_points=graspconfig.compute_contact(contact_z)
        if contact_points.shape[0]>2:
            for contact_point in contact_points:# contact_points指的是物体表面的点
                grasp_moveforward_point = contact_point + 0.01*contact_z#把抓取点前移1cm，如果前移后的抓取与物体碰撞则去除
                # 创建夹爪模型
                box1,box2,box3 = graspconfig.create_gripper_model(grasp_moveforward_point, contact_z , contact_x, contact_y)
                if graspconfig.check_collision(box1) or graspconfig.check_collision(box2) or graspconfig.check_collision(box3):
                    continue
                else:
                    print(contact_point,contact_z,contact_x,contact_y)
                    count += 1
                    break
                 
    end=time.time()
    print(end-start)
    print(count)
    print(graspconfig.point_clouds.shape)