import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter

# 从txt文件中读取数据
with open("area_get/statistics/fpfh.txt", "r") as file:
    data = [float(line.strip()) for line in file]

# 将数据重塑为32x32矩阵，以表示32个元素之间的距离
distance_matrix = np.array(data).reshape(32, 32)

# 对数据进行高斯滤波
sigma = 1  # 高斯滤波的标准差，可以根据需要调整
distance_matrix_filtered = gaussian_filter(distance_matrix, sigma)

# 在除第一列之外的其他列中，将30%的数值随机设置为0-50之间的数值
num_rows, num_cols = distance_matrix_filtered.shape
num_elements_to_change = int(0.3 * num_rows)
for col in range(1, num_cols):
    row_indices = np.random.choice(num_rows, num_elements_to_change, replace=False)
    distance_matrix_filtered[row_indices, col] = np.random.uniform(0, 50, num_elements_to_change)

# 将上三角部分设置为np.nan
distance_matrix_filtered[np.triu_indices(distance_matrix_filtered.shape[0], k=1)] = np.nan

# 再次将对角线设置为0
np.fill_diagonal(distance_matrix_filtered, 0)

# 创建颜色填充格子图
im = plt.imshow(distance_matrix_filtered, cmap='viridis_r')  # 使用反向的viridis颜色映射，这样颜色越深代表距离越远

# 添加颜色条并调整大小
cbar = plt.colorbar(im, shrink=0.8, aspect=20)

# 调整颜色条刻度标签的大小
cbar.ax.tick_params(labelsize=30)

plt.title('经过高斯滤波和随机数修改的颜色填充格子图')
plt.xlabel('元素索引')
plt.ylabel('元素索引')

# 设置横轴和纵轴上的刻度标签
plt.xticks(np.arange(0, 32, 5))
plt.yticks(np.arange(0, 32, 5))

# 调整刻度标签的大小
plt.tick_params(axis='both', which='major', labelsize=30)

plt.show()  # 显示图像