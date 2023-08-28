import numpy as np
import open3d as o3d
import fcl
from fcl import CollisionObject, Box, DynamicAABBTreeCollisionManager, CollisionData, defaultCollisionCallback

def load_obj(file_path):
    mesh = o3d.io.read_triangle_mesh(file_path)
    return mesh

def create_gripper_model(box1_width=0.03, box1_height=0.07, box1_depth=0.01, beam_height=0.02, box3_depth=0.03,
                         distance=0.08, position=(0, 0, 0), gripper_vector=(0, 0, 1),
                         contact_x=(1, 0, 0), contact_y=(0, 1, 0)):
    box1 = o3d.geometry.TriangleMesh.create_box(width=box1_width, height=box1_depth, depth=box1_height)
    box1.translate((-beam_height / 2, -distance / 2 - box1_depth, -box3_depth / 2))
    box2 = o3d.geometry.TriangleMesh.create_box(width=box1_width, height=box1_depth, depth=box1_height)
    box2.translate((-beam_height / 2, distance / 2, -box3_depth / 2))

    box3 = o3d.geometry.TriangleMesh.create_box(width=box3_depth, height=distance, depth=beam_height)
    box3.translate(((box1_width - box3_depth) / 2 - beam_height / 2, -distance / 2, -box3_depth / 2))

    gripper = box1 + box2 + box3

    # 计算旋转
    rotation_matrix = np.column_stack((np.array(contact_x), np.array(contact_y), np.array(gripper_vector)))
    # 旋转并移动夹爪
    gripper.rotate(rotation_matrix)
    gripper.translate(position)
    return gripper

def check_collision(object_mesh, gripper_mesh):
    object_fcl = CollisionObject(Box(*object_mesh.get_max_bound() - object_mesh.get_min_bound()))
    object_fcl.setTranslation(object_mesh.get_center())
    gripper_fcl = CollisionObject(Box(*gripper_mesh.get_max_bound() - gripper_mesh.get_min_bound()))
    gripper_fcl.setTranslation(gripper_mesh.get_center())

    collision_manager = DynamicAABBTreeCollisionManager()
    collision_manager.registerObject(object_fcl)
    collision_manager.registerObject(gripper_fcl)
    collision_manager.setup()

    collision_data = CollisionData()
    collision_manager.collide(collision_data, defaultCollisionCallback)
    return collision_data.result.is_collision

def visualize(object_mesh, gripper_mesh):
    object_mesh.paint_uniform_color([1, 0, 0])  # 红色表示物体
    gripper_mesh.paint_uniform_color([0, 1, 0])  # 绿色表示夹爪

    o3d.visualization.draw_geometries([object_mesh, gripper_mesh])

def main():
    # 加载OBJ文件
    obj_file = "area_get/object/nontextured.obj"
    object_mesh = load_obj(obj_file)

    # 创建夹爪模型
    gripper_pose = np.array([[1, 0, 0, 0],
                             [0, 1, 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
    gripper_mesh = create_gripper_model()

    # 检测碰撞
    is_collision = check_collision(object_mesh, gripper_mesh)

    if is_collision:
        print("二指夹爪与物体发生碰撞")
    else:
        print("二指夹爪与物体没有发生碰撞")

    # 可视化物体和夹爪
    visualize(object_mesh, gripper_mesh)

if __name__ == "__main__":
    main()



