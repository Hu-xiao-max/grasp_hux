import open3d as o3d
import numpy as np

import re

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
    position = position - 1.3*(box1_height - beam_height) * gripper_vector# 夹爪后移一段距离
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


# 可视化夹爪和点云
def visualize_gripper_and_point_cloud(grippers, point_cloud):
    geometries = grippers + [point_cloud]
    o3d.visualization.draw_geometries(geometries)

# 主程序
if __name__ == "__main__":
    objfilepath='./area_get/object/nontextured.obj'
    points = read_obj_vertices(objfilepath)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    filepath='area_get/output/2023-8-29/23-41-41.txt'#读取抓取点和抓取配置
    grasppoints , graspvector_z , graspvector_x , graspvector_y= graspconfig(filepath)
  
    grippers = [create_gripper(position=pos, gripper_vector=vec,contact_x=vec_x,contact_y=vec_y)\
                 for pos, vec , vec_x ,vec_y in zip(grasppoints, graspvector_z , graspvector_x , graspvector_y)]

    visualize_gripper_and_point_cloud(grippers, point_cloud)

    