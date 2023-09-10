import numpy as np
import matplotlib.pyplot as plt

# 用你自己的文件路径替换 'your_file_path.npy'
file_path = '/home/tencent_go/dataset/train/003_cracker_box.npy'

# 读取npy文件
data = np.load(file_path)

# 计算每一行数据中倒数第二个数（力摩擦）的倒数的一半
inverse_half_friction = [d[-2] for d in data]

# 计算直方图数据
hist, bin_edges = np.histogram(inverse_half_friction, bins='auto')

# 创建柱状图
plt.bar(bin_edges[:-1], hist, width=np.diff(bin_edges))

# 设置图表标题和坐标轴标签
plt.title("Inverse Half Friction Histogram")
plt.xlabel("Value")
plt.ylabel("Frequency")

# 显示柱状图
plt.show()