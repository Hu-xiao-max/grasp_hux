import matplotlib.pyplot as plt

# 读取文件
with open("area_get/output/2023-09-01/21-44-20.txt", "r") as f:
    lines = f.readlines()

# 提取每行倒数第三个数字
data = [float(line.split()[-3]) for line in lines]

# 绘制柱形图
plt.hist(data, bins='auto', edgecolor='k')
plt.xlabel("Value")
plt.ylabel("Frequency")
plt.title("Histogram of the Third Last Number in Each Line")
plt.show()