import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 28})
filepath='area_get/output/2023-09-07/01-05/011_banana.txt'

# 读取文件
with open(filepath, "r") as f:
    lines = f.readlines()

# 提取每行倒数第三个数字
data = [float(line.split()[-3]) for line in lines]

# 绘制柱形图
plt.hist(data, bins='auto', edgecolor='k', color='cyan')
# color='blue' 'red', 'green', 'cyan', 'magenta', 'yellow', and 'black'
plt.xlabel("Value")
plt.ylabel("Frequency")
plt.title("Score")
plt.show()