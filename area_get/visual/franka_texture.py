import open3d as o3d

def main():
    # 读取夹爪模型
    gripper = o3d.io.read_triangle_mesh("./area_get/object/fanka.obj")
    gripper.compute_vertex_normals()


    # # 读取物体模型
    # obj = o3d.io.read_triangle_mesh("/home/tencent_go/dataset/objs/plate_large.obj")
    # obj.compute_vertex_normals()

    # 创建坐标系
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=[0, 0, 0])
    # # 可视化
    # o3d.visualization.draw_geometries([gripper, obj])
    o3d.visualization.draw_geometries([gripper, coordinate_frame])

def main1():
    # # # 读取物体模型
    obj = o3d.io.read_triangle_mesh("/home/tencent_go/dataset/objs/plate_large.obj")
    obj.compute_vertex_normals()
    # # # 可视化

    o3d.visualization.draw_geometries([obj])


if __name__ == "__main__":
    main1()