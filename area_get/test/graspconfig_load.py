# 读取txt文件
with open('area_get/output/2023-08-23/15-28-40.txt', 'r') as file:
    data = file.read()

# 将数据分割成行
rows = data.split('\n')

# 初始化两个新的列表
list1 = []
list2 = []

# 遍历每一行（过滤掉空行）
for row in filter(None, rows):
    # 将行分割成两个列表
    l1, l2 = row.split('] [')

    # 去掉多余的字符并将字符串分割成数字列表
    l1 = l1.replace('[', '').split()
    l2 = l2.replace(']', '').split()

    # 将数字字符串转换为浮点数
    l1 = [float(num) for num in l1]
    l2 = [float(num) for num in l2]

    # 将两个列表分别添加到新列表中
    list1.append(l1)
    list2.append(l2)

print(list1)
print(list2)