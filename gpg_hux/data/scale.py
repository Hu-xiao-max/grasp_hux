import numpy as np

def scale_pcd(filename, scale_factor):
    # 读取PCD文件
    with open(filename, 'r') as f:
        lines = f.readlines()

    # 找到数据起始行
    for i, line in enumerate(lines):
        if line.startswith('DATA'):
            start_idx = i + 1
            break

    # 读取点云数据
    data = []
    for line in lines[start_idx:]:
        data.append([float(x) for x in line.split()])

    # 将点云数据转换为NumPy数组
    data = np.array(data)

    # 缩放点云数据
    scaled_data = data.copy()
    scaled_data=scaled_data*scale_factor

    # 将缩放后的点云数据写回PCD文件
    with open(filename, 'w') as f:
        for line in lines[:start_idx]:
            f.write(line)
        for row in scaled_data:
            f.write(' '.join(map(str, row)) + '\n')

# 示例：缩放一个PCD文件
filename = "rabbit1.pcd"  # 请替换为实际的PCD文件名
scale_factor = 0.01  # 缩放因子
scale_pcd(filename, scale_factor)