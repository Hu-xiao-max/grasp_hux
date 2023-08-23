import open3d as o3d
import numpy as np

def create_gripper(box1_width=0.03, box1_height=0.07, box1_depth=0.01, beam_height=0.02,box3_depth = 0.03, distance=0.08, position=(0, 0, 0), gripper_vector=(0, 0, 1),color=(0.5, 0.5, 0.5)):
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
    
    # 将TriangleMesh转换为LineSet以实现半透明效果
    edges = o3d.geometry.LineSet.create_from_triangle_mesh(gripper)
    edges.colors = o3d.utility.Vector3dVector(np.tile(color, (len(edges.lines), 1)))
    return edges

# 创建一个随机点云
def create_random_point_cloud(num_points=1000):
    points = np.random.rand(num_points, 3)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    return point_cloud

# 可视化夹爪和点云
def visualize_gripper_and_point_cloud(grippers, point_cloud, line_width=1.0):
    geometries = grippers + [point_cloud]
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    for geometry in geometries:
        vis.add_geometry(geometry)
    render_option = vis.get_render_option()
    render_option.line_width = line_width
    vis.run()
    vis.destroy_window()

# 主程序
if __name__ == "__main__":
    positions = [(0, 0, 0), (0.2, 0.2, 0), (0.4, 0.4, 0)]
    gripper_vectors = [(0, 0, 1), (0, 1, 0), (1, 0, 0)]
    grippers = [create_gripper(position=pos, gripper_vector=vec) for pos, vec in zip(positions, gripper_vectors)]
    point_cloud = create_random_point_cloud()
    visualize_gripper_and_point_cloud(grippers, point_cloud, line_width=1.0)