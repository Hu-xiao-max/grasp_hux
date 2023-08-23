import open3d as o3d
import numpy as np

def compute_fpfh(point_cloud, search_radius=0.05, kdtree_search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)):
    # 从输入点云计算法线
    point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=search_radius, max_nn=30))
    
    # 计算FPFH特征
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(point_cloud, kdtree_search_param)
    return fpfh

def fpfh_similarity(fpfh1, fpfh2, distance_metric='euclidean'):
    if distance_metric == 'euclidean':
        # 计算欧氏距离
        distances = np.sqrt(np.sum((fpfh1.data.T - fpfh2.data.T)**2, axis=1))
    elif distance_metric == 'cosine':
        # 计算余弦距离
        distances = 1 - np.dot(fpfh1.data.T, fpfh2.data.T) / (np.linalg.norm(fpfh1.data.T, axis=1) * np.linalg.norm(fpfh2.data.T, axis=1))
    else:
        raise ValueError("Unsupported distance metric: {}".format(distance_metric))
    
    # 计算平均距离
    mean_distance = np.mean(distances)
    return mean_distance

# 用NumPy数组表示的点云数据
points1 = np.random.rand(1000, 3)
points2 = np.random.rand(1000, 3)

# 从NumPy数组创建点云对象
point_cloud1 = o3d.geometry.PointCloud()
point_cloud1.points = o3d.utility.Vector3dVector(points1)

point_cloud2 = o3d.geometry.PointCloud()
point_cloud2.points = o3d.utility.Vector3dVector(points2)

# 计算FPFH特征
fpfh1 = compute_fpfh(point_cloud1)
fpfh2 = compute_fpfh(point_cloud2)

# 计算FPFH特征之间的相似度
similarity = fpfh_similarity(fpfh1, fpfh2)
print("FPFH similarity:", similarity)
#分数越低，表示两个点云越相似