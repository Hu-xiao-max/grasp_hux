import open3d as o3d
import trimesh
import numpy as np
import pymeshfix

def create_box(center, extents):
    box = o3d.geometry.TriangleMesh.create_box(*extents)
    box.translate(center - 0.5 * np.array(extents))
    return box

def visualize_mesh(mesh_list):
    o3d.visualization.draw_geometries(mesh_list)

def main():
    # 读取 .obj 文件
    obj_mesh = o3d.io.read_triangle_mesh("path/to/your/obj/file.obj")

    # 创建长方体
    box_center = np.array([0, 0, 0])
    box_extents = np.array([1, 2, 3])
    box_mesh = create_box(box_center, box_extents)

    # 将 Open3D mesh 转换为 trimesh
    obj_trimesh = trimesh.Trimesh(vertices=np.asarray(obj_mesh.vertices), faces=np.asarray(obj_mesh.triangles))
    box_trimesh = trimesh.Trimesh(vertices=np.asarray(box_mesh.vertices), faces=np.asarray(box_mesh.triangles))

    # 碰撞检测
    meshfix = pymeshfix.MeshFix(obj_trimesh)
    meshfix.repair(verbose=False, joincomp=True, remove_smallest_components=False)
    obj_trimesh = meshfix.mesh

    meshfix = pymeshfix.MeshFix(box_trimesh)
    meshfix.repair(verbose=False, joincomp=True, remove_smallest_components=False)
    box_trimesh = meshfix.mesh

    intersection = obj_trimesh.intersection(box_trimesh)

    # 可视化
    if len(intersection.vertices) > 0:
        print("碰撞发生")
        box_mesh.paint_uniform_color([1, 0, 0])  # 红色表示碰撞
    else:
        print("无碰撞")
        box_mesh.paint_uniform_color([0, 1, 0])  # 绿色表示无碰撞

    visualize_mesh([obj_mesh, box_mesh])

if __name__ == "__main__":
    main()