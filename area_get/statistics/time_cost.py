import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 28})
# 数据
indexes = [1, 2]
values = [1.2, 10.6]
colors = ['yellow', 'cyan']

# 创建横向柱状图，设置宽度为0.4
plt.barh(indexes, values, color=colors, height=0.4)

# 设置纵轴标签
plt.yticks(indexes, ['Ours', 'Other Methods Average'])

# 设置横轴标签
plt.xlabel('Hours')

# 设置图标题
plt.title('Time Cost')

# 显示图形
plt.show()