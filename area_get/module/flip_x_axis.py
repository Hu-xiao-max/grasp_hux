import numpy as np
'''
在这个示例中，flip_x_axis 函数接收点云数据（points），
然后创建一个点云数据的副本（flipped_points）。
接着，我们将副本中的 X 坐标乘以 -1，从而使 X 轴朝向相反方向。最后，返回处理后的点云数据。
'''

def flip_x_axis(points):
    """
    使点云区域的 X 轴朝向相反方向。

    参数：
        points (numpy.ndarray): 点云数据，shape 为 (N, 3)。

    返回：
        numpy.ndarray: X 轴朝向相反的点云，shape 为 (N, 3)。
    """
    flipped_points = points.copy()
    flipped_points[:, 0] *= -1
    flipped_points[:, 2] *= -1
    return flipped_points

if __name__ == "__main__":
    # 创建一个示例点云数据
    points = np.array([
        [0, 0, 0],
        [1, 1, 1],
        [2, 2, 2],
        [3, 3, 3],
        [4, 4, 4],
    ])

    # 使 X 轴朝向相反方向
    flipped_points = flip_x_axis(points)
    print("Flipped points:", flipped_points)