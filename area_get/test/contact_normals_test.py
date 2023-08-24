import numpy as np
import math
normals= np.array([1,0,0])
gripper_direction = np.array([0,-1,0])
angles = np.arccos(np.dot(normals, gripper_direction)/math.sqrt(np.dot(normals, normals))) * math.sqrt(np.dot(gripper_direction, gripper_direction))
print(angles)
print(np.radians(180))
print(np.radians(160))
#contact_points = np.asarray(pcd.points)[np.logical_and(np.radians(160) < angles, angles < np.radians(180))]
import numpy as np

def compute_contact(self, gripper_direction):
    # 计算夹爪方向与点云法向量之间的夹角
    gripper_direction = np.array(gripper_direction)
    gripper_direction_norm = np.linalg.norm(gripper_direction)
    normals_norm = np.linalg.norm(self.normals, axis=1)
    
    cos_angles = np.dot(self.normals, gripper_direction) / (normals_norm * gripper_direction_norm)
    angles = np.arccos(cos_angles)
    
    # 定义一个阈值来选择可接触的点（这里假设夹角小于 45 度的点可以接触）
    # 提取可接触的点
    contact_points = np.asarray(self.pcd.points)[np.logical_and(np.radians(160) < angles, angles < np.radians(180))]
    
    # 将可接触的点保存到新的点云对象并可视化
    # contact_points_pcd = o3d.geometry.PointCloud()
    # contact_points_pcd.points = o3d.utility.Vector3dVector(contact_points)
    # o3d.visualization.draw_geometries([contact_points_pcd])
    return contact_points