import numpy as np


class specify_gripper_up_vector():
    def __init__(self):
        self.gripper_up_vector = np.array([0, 0, 1])

    def transform_to_gripper_space(self,points, contact_point, contact_vector, gripper_up_vector):
        # 计算夹爪的局部坐标系基向量
        gripper_z_axis = contact_vector / np.linalg.norm(contact_vector)
        gripper_x_axis = np.cross(gripper_up_vector, gripper_z_axis)
        gripper_x_axis /= np.linalg.norm(gripper_x_axis)
        gripper_y_axis = np.cross(gripper_z_axis, gripper_x_axis)

        # 计算点云到夹爪局部坐标系的变换矩阵
        rotation_matrix = np.array([gripper_x_axis, gripper_y_axis, gripper_z_axis]).T
        translation_vector = -np.dot(rotation_matrix, contact_point)

        # 将点云变换到夹爪局部坐标系
        return np.dot(points + translation_vector, rotation_matrix)

    def points_in_gripper_space(self,points, contact_point, contact_vector, gripper_up_vector, gripper_size):
        # 将点云变换到夹爪局部坐标系
        points_transformed = self.transform_to_gripper_space(points, contact_point, contact_vector, gripper_up_vector)

        # 计算夹爪长方体的 AABB
        gripper_aabb_min = -gripper_size / 2
        gripper_aabb_max = gripper_size / 2

        # 提取夹爪长方体空间内的点云
        mask = np.all((points_transformed >= gripper_aabb_min) & (points_transformed <= gripper_aabb_max), axis=1)
        return points[mask]
    
    def main(self):
        # 创建一个示例点云数据
        points = np.array([
            [0, 0, 0],
            [1, 1, 1],
            [2, 2, 2],
            [3, 3, 3],
            [4, 4, 4],
        ])

        # 设置接触点和接触向量
        contact_point = np.array([2, 2, 2])
        contact_vector = np.array([1, 0, 0])

        # 设置夹爪的朝上向量
        gripper_up_vector = np.array([0, 0, 1])

        # 设置夹爪长方体的尺寸
        gripper_size = np.array([2, 2, 2])

        # 计算夹爪长方体空间内的点云
        points_in_gripper = self.points_in_gripper_space(points, contact_point, contact_vector, gripper_up_vector, gripper_size)
        print("Points in gripper space:", points_in_gripper)


class randomization_gripper_up_vector():
    def __init__(self):
        self.gripper_up_vector = None

    def transform_to_gripper_space(self,points, contact_point, contact_vector):
        # 计算夹爪的局部坐标系基向量
        gripper_z_axis = contact_vector / np.linalg.norm(contact_vector)
        
        # 随机选择一个与接触向量垂直的向量作为夹爪的 X 轴
        random_vector = np.random.randn(3)
        gripper_x_axis = np.cross(random_vector, gripper_z_axis)
        
        # 如果 X 轴和 Z 轴平行，重新选择一个随机向量
        while np.linalg.norm(gripper_x_axis) < 1e-6:
            random_vector = np.random.randn(3)
            gripper_x_axis = np.cross(random_vector, gripper_z_axis)
            
        print(gripper_x_axis)
        
        gripper_x_axis /= np.linalg.norm(gripper_x_axis)
        gripper_y_axis = np.cross(gripper_z_axis, gripper_x_axis)

        # 计算点云到夹爪局部坐标系的变换矩阵
        rotation_matrix = np.array([gripper_x_axis, gripper_y_axis, gripper_z_axis]).T
        translation_vector = -np.dot(rotation_matrix, contact_point)

        # 将点云变换到夹爪局部坐标系
        return np.dot(points + translation_vector, rotation_matrix)

    def points_in_gripper_space(self,points, contact_point, contact_vector, gripper_size):
        # 将点云变换到夹爪局部坐标系
        points_transformed = self.transform_to_gripper_space(points, contact_point, contact_vector)

        # 计算夹爪长方体的 AABB
        gripper_aabb_max = gripper_size / 2
        gripper_aabb_min = -gripper_size / 2
        gripper_aabb_max[2]=gripper_aabb_max[2]-gripper_aabb_min[2]
        gripper_aabb_min[2]=0
        #更新为夹爪坐标系在夹爪中心
        #print(gripper_aabb_max,gripper_aabb_min)

        # 提取夹爪长方体空间内的点云
        mask = np.all((points_transformed >= gripper_aabb_min) & (points_transformed <= gripper_aabb_max), axis=1)
        return points[mask]
    
    def main(self):
        # 创建一个示例点云数据
        points = np.array([
            [0, 0, 0],
            [1, 1, 1],
            [2, 2, 2],
            [3, 3, 3],
            [4, 4, 4],
        ])

        # 设置接触点和接触向量
        contact_point = np.array([2, 2, 2])
        contact_vector = np.array([1, 0, 0])

        # 设置夹爪长方体的尺寸
        gripper_size = np.array([2, 2, 2])

        # 计算夹爪长方体空间内的点云
        points_in_gripper = self.points_in_gripper_space(points, contact_point, contact_vector, gripper_size)
        print("Points in gripper space:", points_in_gripper)


if __name__ == "__main__":
    a=specify_gripper_up_vector()
    a.main()
    
    b=randomization_gripper_up_vector()
    b.main()