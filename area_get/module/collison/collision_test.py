import numpy as np
import open3d as o3d
import fcl
from fcl import CollisionObject, Box, DynamicAABBTreeCollisionManager, CollisionData, defaultCollisionCallback


class collision():
    def __init__(self):

        self.gripper = self.create_gripper()
        self.point_cloud=self.read_obj_vertices()
        self.filepath = "area_get/object/nontextured.obj"

    def load_obj(self,file_path):
        mesh = o3d.io.read_triangle_mesh(file_path)
        return mesh
    
    def read_obj_vertices(self):
        vertices = []

        with open("area_get/object/nontextured.obj", 'r') as file:
            for line in file:
                if line.startswith('v '):
                    vertex = line.strip().split(' ')[1:]
                    vertex = [float(coord) for coord in vertex]
                    vertices.append(vertex)
            vertices=np.array(vertices)

        return vertices


    def create_gripper(self,box1_width=0.03, box1_height=0.07, box1_depth=0.01, beam_height=0.02, box3_depth=0.03,
                            distance=0.08):
        box1 = o3d.geometry.TriangleMesh.create_box(width=box1_width, height=box1_depth, depth=box1_height)
        box1.translate((-beam_height / 2, -distance / 2 - box1_depth, -box3_depth / 2))
        box2 = o3d.geometry.TriangleMesh.create_box(width=box1_width, height=box1_depth, depth=box1_height)
        box2.translate((-beam_height / 2, distance / 2, -box3_depth / 2))

        box3 = o3d.geometry.TriangleMesh.create_box(width=box3_depth, height=distance, depth=beam_height)
        box3.translate(((box1_width - box3_depth) / 2 - beam_height / 2, -distance / 2, -box3_depth / 2))

        origin_gripper = box1 + box2 + box3

        return origin_gripper
    
    def create_box(self,box1_width=0.03, box1_height=0.07, box1_depth=0.01, beam_height=0.02, box3_depth=0.03,
                            distance=0.08,position=(-0.02, 0, -0.04),gripper_vector=(0, 0, 1),contact_x=(0, -1, 0), contact_y=( 1, 0 , 0)):
        box1 = o3d.geometry.TriangleMesh.create_box(width=box1_width, height=box1_depth, depth=box1_height)
        box1.translate((-beam_height / 2, -distance / 2 - box1_depth, -box3_depth / 2))
        box2 = o3d.geometry.TriangleMesh.create_box(width=box1_width, height=box1_depth, depth=box1_height)
        box2.translate((-beam_height / 2, distance / 2, -box3_depth / 2))

        box3 = o3d.geometry.TriangleMesh.create_box(width=box3_depth, height=distance, depth=beam_height)
        box3.translate(((box1_width - box3_depth) / 2 - beam_height / 2, -distance / 2, -box3_depth / 2))
        rotation_matrix = np.column_stack((np.array(contact_x), np.array(contact_y), np.array(gripper_vector)))

        box1.rotate(rotation_matrix)
        box1.translate(position)
        box2.rotate(rotation_matrix)
        box2.translate(position)
        box3.rotate(rotation_matrix)
        box3.translate(position)



        return box1,box2,box3


    def create_gripper_model(self,position=(-0.02, 0, -0.04),gripper_vector=(0, 0, 1),
                            contact_x=(0, -1, 0), contact_y=(1, 0 , 0)):
        # 计算旋转
        rotation_matrix = np.column_stack((np.array(contact_x), np.array(contact_y), np.array(gripper_vector)))
        # 旋转并移动夹爪
        gripper= self.gripper
        gripper.rotate(rotation_matrix)
        gripper.translate(position)
    
    
        return gripper
    
    def create_triangle_mesh_from_point_cloud(self):
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(self.point_cloud)
        return mesh

    def check_collision(self, gripper_mesh):
        object_mesh = self.create_triangle_mesh_from_point_cloud()
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

    def visualize(self,object_mesh, gripper_mesh):
        object_mesh.paint_uniform_color([1, 0, 0])  # 红色表示物体
        gripper_mesh.paint_uniform_color([0, 1, 0])  # 绿色表示夹爪

        o3d.visualization.draw_geometries([object_mesh, gripper_mesh])

    def main(self):
        # 加载OBJ文件
        obj_file = "area_get/object/nontextured.obj"
        object_mesh = self.load_obj(obj_file)

        # 创建夹爪模型
        gripper_mesh = self.create_gripper_model()
        box1,box2,box3= self.create_box()

        # 检测碰撞

        if self.check_collision(box2) or self.check_collision(box2) or self.check_collision(box3):
            print("二指夹爪与物体发生碰撞")
        else:
            print("二指夹爪与物体没有发生碰撞")

        # 可视化物体和夹爪
        self.visualize(object_mesh, gripper_mesh)

if __name__ == "__main__":
    v= collision()
    v.main()