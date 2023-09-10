import matplotlib.pyplot as plt

# 设置全局字体大小
plt.rcParams.update({'font.size': 30})

# 从txt文件中读取数据
with open("/home/tencent_go/paper/grasp_hux/area_get/module/similar_points/fpfh.txt", "r") as file:
    data = [float(line.strip()) for line in file]

# 创建散点图
plt.scatter(range(len(data)), data)
plt.xlabel('Index')
plt.ylabel('Norm Distance')
plt.title('Difference between novel and base')
plt.show()