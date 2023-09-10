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
    gripper.translate(position)
    return gripper

def load_obj(file_path):
    mesh = o3d.io.read_triangle_mesh(file_path)
    return mesh


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

def point_inside_gripper(point, gripper, tolerance=1e-6):
    aabb = gripper.get_axis_aligned_bounding_box()
    point_min = aabb.min_bound - tolerance
    point_max = aabb.max_bound + tolerance
    return np.all(point > point_min) and np.all(point < point_max)

def visualize_gripper_and_point_cloud(grippers, point_cloud):
    # 创建坐标系
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])

    # 设置点云颜色
    colors = []
    for point in point_cloud.points:
        in_gripper_space = False
        for gripper in grippers:
            if point_inside_gripper(point, gripper):
                in_gripper_space = True
                break
        if in_gripper_space:
            colors.append([1, 1, 1])  # 红色
        else:
            colors.append([1, 1, 1])  # 1 1 0黄色1,1,1是白色
    point_cloud.colors = o3d.utility.Vector3dVector(colors)

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
    # objfilepath='./area_get/object/nontextured.obj'
    objfilepath='/home/tencent_go/dataset/objs/plate_large.obj'
    points = read_obj_vertices(objfilepath)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    filepath='area_get/output/2023-08-24/11-32-16.txt'
    #读取抓取点和抓取配置
    # Load the existing gripper model
    gripper_obj_path = './area_get/object/fanka.obj'
    gripper_mesh = load_obj(gripper_obj_path)

    # Apply transformations to the gripper mesh
    pos = [-0.05, 0.02, -0.04]
    vec = [1, 0, 0]
    contact_x = [0, -1, 0]
    contact_y = [0, 0, 1]
    rotation_matrix = np.column_stack((np.array(contact_x), np.array(contact_y), np.array(vec)))
    gripper_mesh.rotate(rotation_matrix)
    gripper_mesh.translate(pos)

    grippers = [gripper_mesh]

    visualize_gripper_and_point_cloud(grippers, point_cloud)
