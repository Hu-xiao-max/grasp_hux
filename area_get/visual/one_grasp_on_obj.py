import open3d as o3d
import numpy as np

def graspconfig(filepath):
    with open(filepath, 'r') as file:
        data = file.read()
    # 将数据分割成行
    rows = data.split('\n')
    # 初始化两个新的列表
    list1 = []
    list2 = []
    # 遍历每一行（过滤掉空行）
    for row in filter(None, rows):
        # 将行分割成两个列表
        l1, l2 = row.split('] [')
        # 去掉多余的字符并将字符串分割成数字列表
        l1 = l1.replace('[', '').split()
        l2 = l2.replace(']', '').split()
        # 将数字字符串转换为浮点数
        l1 = [float(num) for num in l1]
        l2 = [float(num) for num in l2]
        # 将两个列表分别添加到新列表中
        list1.append(l1)
        list2.append(l2)
    return list1, list2

# 创建一个简单的二指夹爪
def create_gripper(box1_width=0.03, box1_height=0.07, box1_depth=0.01, beam_height=0.02,box3_depth = 0.03, distance=0.08, position=(0, 0, 0), gripper_vector=(0, 0, 1)):
    box1 = o3d.geometry.TriangleMesh.create_box(width=box1_depth, height=box1_height, depth=box1_width)
    box1.translate((-distance / 2 - box1_depth, 0, 0))
    box2 = o3d.geometry.TriangleMesh.create_box(width=box1_depth, height=box1_height, depth=box1_width)
    box2.translate((distance / 2, 0, 0))
    
    box3 = o3d.geometry.TriangleMesh.create_box(width=distance, height=beam_height, depth=box3_depth)
    box3.translate((-distance / 2,box1_height - beam_height, (box1_width- box3_depth) / 2))
    
    gripper = box1 + box2 + box3
    
    # 计算旋转
    gripper_vector = np.array(gripper_vector) / np.linalg.norm(gripper_vector)
    z_axis = np.array([0, 0, 1])
    rotation_axis = np.cross(z_axis, gripper_vector)
    rotation_angle = np.arccos(np.dot(z_axis, gripper_vector))
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_axis * rotation_angle)
    
    # 旋转并移动夹爪
    gripper.rotate(rotation_matrix)
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

def visualize_gripper_and_point_cloud(grippers, point_cloud):
    # 创建坐标系
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])

    # 添加坐标系、夹爪和点云到要显示的几何体列表中
    geometries = grippers + [point_cloud, frame]

    # 创建一个可视化器
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # 将几何体添加到可视化器中
    for geometry in geometries:
        vis.add_geometry(geometry)

    # 设置视点，使坐标系可见
    vis.get_render_option().point_size = 2.0
    vis.get_view_control().set_zoom(0.6)

    # 运行可视化器
    vis.run()

    # 关闭可视化器
    vis.destroy_window()

# 主程序
if __name__ == "__main__":
    objfilepath='./area_get/object/nontextured.obj'
    points = read_obj_vertices(objfilepath)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    filepath='area_get/output/2023-08-24/11-32-16.txt'
    #读取抓取点和抓取配置
    # grasppoints , graspvector_z = graspconfig(filepath)

    grasppoints = [[-0.01,0,-0.02],[-0.01,-0.1,-0.03]]
    graspvector_z = [[0,1,0],[0,1,0]]

    # pos = [0,0,0]
    # vec = [1,0,0]
    #grippers = [create_gripper(position=pos, gripper_vector=vec)]

    grippers = [create_gripper(position=pos, gripper_vector=vec) for pos, vec in zip(grasppoints[0:2], graspvector_z[0:2])]
    

    visualize_gripper_and_point_cloud(grippers, point_cloud)