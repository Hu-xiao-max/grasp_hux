import numpy as np




def transform_points(points, origin, y_axis):
    # 计算新坐标系的 Z 轴（垂直于 Y 轴的任意向量）
    z_axis = np.cross(y_axis, np.array([1, 0, 0]))
    
    if np.linalg.norm(z_axis) < 1e-6:  # 如果 Y 轴与 X 轴平行
        z_axis = np.cross(y_axis, np.array([0, 0, 1]))

    # 计算新坐标系的 X 轴（垂直于 Y 轴和 Z 轴）
    x_axis = np.cross(y_axis, z_axis)
    
    # 归一化基向量
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    z_axis = z_axis / np.linalg.norm(z_axis)
    #print(x_axis,y_axis,z_axis)
    # 构建变换矩阵
    translation_vector = -origin
    rotation_matrix = np.array([x_axis, y_axis, z_axis]).T
    

    # 将点云坐标转换为新坐标系
    transformed_points = np.dot(points.astype(np.float64) + translation_vector, rotation_matrix)
    return transformed_points



if __name__ == "__main__":
    # 创建一个示例点云数据
    points = np.array([
        [0, 0, 0],
        [1, 1, 1],
        [2, 2, 2],
        [3, 3, 3],
        [4, 4, 4],
    ])

    # 设置新坐标系的原点和 Y 轴
    origin = np.array([2, 2, 2])
    y_axis = np.array([0, 1, 0])

    # 将点云坐标转换为新坐标系
    transformed_points = transform_points(points, origin, y_axis)
    print( transformed_points)