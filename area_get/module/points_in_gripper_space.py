import numpy as np

def transform_to_gripper_space(points, contact_point, contact_vector, gripper_up_vector):
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

def points_in_gripper_space(points, contact_point, contact_vector, gripper_up_vector, gripper_size):
    # 将点云变换到夹爪局部坐标系
    points_transformed = transform_to_gripper_space(points, contact_point, contact_vector, gripper_up_vector)

    # 计算夹爪长方体的 AABB
    gripper_aabb_min = -gripper_size / 2
    gripper_aabb_max = gripper_size / 2

    # 提取夹爪长方体空间内的点云
    mask = np.all((points_transformed >= gripper_aabb_min) & (points_transformed <= gripper_aabb_max), axis=1)
    return points[mask]

if __name__ == "__main__":
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
    points_in_gripper = points_in_gripper_space(points, contact_point, contact_vector, gripper_up_vector, gripper_size)
    print("Points in gripper space:", points_in_gripper)