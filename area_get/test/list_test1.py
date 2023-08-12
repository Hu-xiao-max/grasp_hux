numbers = [1, 2, 3, 2, 4, 5, 4, 6, 1]
# 创建一个字典，键为数值，值为出现次数
count_dict = {}

# 遍历数值，统计每个数值的出现次数
for num in numbers:
    if num in count_dict:
        count_dict[num] += 1
    else:
        count_dict[num] = 1

# 打印结果
for num, count in count_dict.items():
    if count > 1:
        print(f"{num} is repeated {count} times")