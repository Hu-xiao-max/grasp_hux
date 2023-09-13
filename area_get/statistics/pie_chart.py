import matplotlib.pyplot as plt

# 数据准备
labels = ['Great', 'Non stable', 'Failure']
sizes = [ 24.6, 32.8, 42.6]
colors = ['#66b3ff', '#99ff99', '#ffcc99']
explode = ( 0.05, 0.05, 0.05)  # 用于突出显示每个部分的偏移

# 设置字体大小
font_size = 30
textprops = {"fontsize": font_size}

# 创建饼图
fig, ax = plt.subplots()
ax.pie(sizes, explode=explode, colors=colors, autopct='%1.1f%%', startangle=90, textprops=textprops)

# 使饼图保持一个圆形
ax.axis('equal')

# 显示图形
plt.show()