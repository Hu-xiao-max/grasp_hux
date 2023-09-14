import numpy as np
import matplotlib.pyplot as plt

# 从txt文件中读取数据
with open("area_get/statistics/fpfh.txt", "r") as file:
    data = [float(line.strip()) for line in file]

# 将数据重塑为32x32矩阵，以表示32个元素之间的距离
distance_matrix = np.array(data).reshape(32, 32)


# 将对角线元素设置为0
np.fill_diagonal(distance_matrix, 0)

# 创建颜色填充格子图
plt.imshow(distance_matrix, cmap='viridis_r')  # 使用反向的viridis颜色映射，这样颜色越深代表距离越远
plt.colorbar()  # 添加颜色条
plt.title('颜色填充格子图')
plt.xlabel('元素索引')
plt.ylabel('元素索引')
plt.show()  # 显示图像