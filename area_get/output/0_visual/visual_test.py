import matplotlib.pyplot as plt

# 从文件中读取数据
with open("./output/2023-08-13/test.txt", "r") as file:
    lines = file.readlines()

datasets = []

for line in lines:
    raw_dict = line.split('{')[1].split('}')[0]
    data_dict = eval('{' + raw_dict + '}')
    datasets.append(data_dict)

x = []
y = []

for dataset in datasets:
    for key, value in dataset.items():
        x.append(key)
        y.append(len(value))

plt.bar(x, y)
plt.xlabel('Dict Keys')
plt.ylabel('Element Count')
plt.title('Repeat of Data in crackerbox')
plt.show()