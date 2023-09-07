import open3d as o3d
import numpy as np
import re
import time
from scipy.spatial.distance import cdist

def graspconfig(filepath):
    with open(filepath, 'r') as file:
        data = file.read()
    # 将数据分割成行
    rows = data.split('\n')
    # 初始化四个新的列表
    list1 = []
    list2 = []
    list3 = []
    list4 = []
    # 遍历每一行（过滤掉空行）
    for row in filter(None, rows):
        # 使用正则表达式匹配四个向量
        vectors = re.findall(r'\[([^\]]+)\]', row)
        # 去掉多余的字符并将字符串分割成数字列表
        l1 = vectors[0].split()
        l2 = vectors[1].split()
        l3 = vectors[2].split()
        l4 = vectors[3].split()
        # 将数字字符串转换为浮点数
        l1 = [float(num) for num in l1]
        l2 = [float(num) for num in l2]
        l3 = [float(num) for num in l3]
        l4 = [float(num) for num in l4]
        # 将四个列表分别添加到新列表中
        list1.append(l1)
        list2.append(l2)
        list3.append(l3)
        list4.append(l4)
    return list1, list2, list3, list4

# 创建一个简单的二指夹爪
def create_gripper(box1_width=0.03, box1_height=0.07, box1_depth=0.01, beam_height=0.02,box3_depth = 0.03, \
                   distance=0.08, position=(0, 0, 0), gripper_vector=(0, 0, 1),contact_x=(1,0,0),contact_y=(0,1,0)):
    box1 = o3d.geometry.TriangleMesh.create_box(width=box1_width, height=box1_depth, depth=box1_height)
    box1.translate(( - beam_height/2,-distance / 2 - box1_depth , -box3_depth/2))
    box2 = o3d.geometry.TriangleMesh.create_box(width=box1_width, height=box1_depth, depth=box1_height)
    box2.translate(( - beam_height/2,distance / 2, -box3_depth/2))
    
    box3 = o3d.geometry.TriangleMesh.create_box(width=box3_depth, height=distance, depth=beam_height)
    box3.translate(((box1_width- box3_depth) / 2 - beam_height/2, -distance / 2, -box3_depth/2 ))
    
    gripper = box1 + box2 + box3

    # 计算旋转
    rotation_matrix=np.column_stack((np.array(contact_x), np.array(contact_y), np.array(gripper_vector)))
    # 旋转并移动夹爪
    gripper.rotate(rotation_matrix)

    gripper_vector = np.array(gripper_vector)
    gripper.translate(position)
    return gripper


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


def is_point_inside_aabb(aabb, point, tolerance=1e-3):
    min_bound = aabb.min_bound - tolerance
    max_bound = aabb.max_bound + tolerance
    return np.all(min_bound <= point) and np.all(point <= max_bound)

def get_points_inside_gripper(gripper, point_cloud, tolerance=1e-3):
    gripper_aabb = gripper.get_axis_aligned_bounding_box()
    gripper_points = []

    for point in point_cloud.points:
        if is_point_inside_aabb(gripper_aabb, point, tolerance):
            gripper_points.append(point)

    return gripper_points

def compute_fpfh(points, search_radius=0.05, kdtree_search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)):
    # Convert the points numpy array to an Open3D PointCloud object
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    
    # Estimate normals for the point cloud
    point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=search_radius, max_nn=30))
    
    # Compute FPFH features
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(point_cloud, kdtree_search_param)
    return fpfh

def fpfh_similarity(fpfh1, fpfh2, distance_metric='euclidean'):
    if distance_metric == 'euclidean':
        # Compute pairwise Euclidean distances
        distances = cdist(fpfh1.data.T, fpfh2.data.T, metric='euclidean')
    elif distance_metric == 'cosine':
        # Compute pairwise cosine distances
        distances = cdist(fpfh1.data.T, fpfh2.data.T, metric='cosine')
    else:
        raise ValueError("Unsupported distance metric: {}".format(distance_metric))
    
    # Compute the average distance
    mean_distance = np.mean(distances)
    return mean_distance


# 主程序
if __name__ == "__main__":
    start =time.time()
    compare_objfilepath='/home/tencent_go/dataset/ycb-tools/models/ycb/052_extra_large_clamp/poisson/textured.obj'
    objfilepath='./area_get/object/nontextured.obj'

    points = read_obj_vertices(objfilepath)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)


    compare_points = read_obj_vertices(compare_objfilepath)
    compare_point_cloud = o3d.geometry.PointCloud()
    compare_point_cloud.points = o3d.utility.Vector3dVector(compare_points)
    #以下为用来对比的closing region
    grasppoints = [[0,0.01,0.12]]
    graspvector_z = [[0,0,-1]]
    graspvector_x = [[-1,0,0]]
    graspvector_y = [[0,1,0]]
  
    compare_grippers = [create_gripper(position=pos, gripper_vector=vec,contact_x=vec_x,contact_y=vec_y)\
                 for pos, vec , vec_x ,vec_y in zip(grasppoints, graspvector_z , graspvector_x , graspvector_y)]
    
    compare_gripper_points_list = [
        get_points_inside_gripper(compare_gripper, compare_point_cloud) for compare_gripper in compare_grippers
    ]
    

    filepath='area_get/output/2023-09-01/test.txt'#读取抓取点和抓取配置
    grasppoints , graspvector_z , graspvector_x , graspvector_y= graspconfig(filepath)
  
    grippers = [create_gripper(position=pos, gripper_vector=vec,contact_x=vec_x,contact_y=vec_y)\
                 for pos, vec , vec_x ,vec_y in zip(grasppoints, graspvector_z , graspvector_x , graspvector_y)]


    gripper_points_list = [
        get_points_inside_gripper(gripper, point_cloud) for gripper in grippers
    ]
     # Open a new file for writing the results
    with open("area_get/module/similar_points/fpfh_similarity_results.txt", "w") as output_file:
        for t, compare_gripper_points in enumerate(compare_gripper_points_list):
            fpfh1 = compute_fpfh(np.array(compare_gripper_points))
        
            for i, gripper_points in enumerate(gripper_points_list):
                fpfh2 = compute_fpfh(np.array(gripper_points))
                similarity = fpfh_similarity(fpfh1, fpfh2)
                print("FPFH similarity:", similarity)
                # Write the FPFH similarity value to the file
                output_file.write(str(similarity))
                output_file.write('\n')
        # Close the file after writing the results
        output_file.close()


    end = time.time()
    print("Total time taken:", end - start)

            # print(f"Gripper {i + 1}: {len(gripper_points)} points")
            # print(np.array(gripper_points))
    # save_gripper_points_to_txt(gripper_points_list, 'gripper_points.txt')



    