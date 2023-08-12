import open3d as o3d
import numpy as np
import time

start=time.time()

# 加载点云数据（请替换为实际的点云文件路径）
pcd = o3d.io.read_point_cloud('/home/tencent_go/paper/area_get/object/rabbit1.pcd')

# 对点云进行下采样和平滑处理

k=10#k控制的是搜索的neighbour
smoothed_pcd = o3d.geometry.PointCloud.estimate_normals(pcd, search_param=o3d.geometry.KDTreeSearchParamKNN(k))
print(smoothed_pcd)
print(pcd)
# 计算点云的法向量
smoothed_pcd=pcd
normals = np.asarray(smoothed_pcd.normals)
print(normals)
# 定义夹爪方向（这里假设夹爪方向沿着 z 轴正方向）
gripper_direction = np.array([0, 0, 1])

# 计算夹爪方向与点云法向量之间的夹角
angles = np.arccos(np.dot(normals, gripper_direction))
# print(angles)
# 定义一个阈值来选择可接触的点（这里假设夹角小于 45 度的点可以接触）
angle_threshold = np.radians(45)
# print('angle judge',[angles < angle_threshold])

# 提取可接触的点
contact_points = np.asarray(smoothed_pcd.points)[angles < angle_threshold]

end=time.time()

print('time is ',end-start)
# 将可接触的点保存到新的点云对象并可视化
contact_points_pcd = o3d.geometry.PointCloud()
contact_points_pcd.points = o3d.utility.Vector3dVector(contact_points)
o3d.visualization.draw_geometries([contact_points_pcd])


