# Initialize lists to store vectors
list1 = []
list2 = []
list3 = []
list4 = []

# Read data from txt file
with open('area_get/output/2023-08-27/test.txt', 'r') as file:
    for line in file:
        # Remove newline character and split the line by spaces
        line = line.strip().split('] [')
        
        # Extract vectors and store them in the lists
        vec1 = [float(x) for x in line[0][1:].split()]
        vec2 = [float(x) for x in line[1].split()]
        vec3 = [float(x) for x in line[2].split()]
        vec4 = [float(x) for x in line[3][:-1].split()]
        
        list1.append(vec1)
        list2.append(vec2)
        list3.append(vec3)
        list4.append(vec4)

# Print the lists
print("List 1:", list1)
print("List 2:", list2)
print("List 3:", list3)
print("List 4:", list4)


with open('area_get/output/2023-08-27/test.txt', 'r') as file:
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
print("List 1:", list1)
print("List 2:", list2)
