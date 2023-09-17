import matplotlib.pyplot as plt
# 更新默认字体大小
plt.rcParams.update({'font.size': 36})
name = '026_sponge'
filepath1 = 'area_get/output/2023-09-10/17-27/'+name+'.txt'
filepath2 = 'area_get/output/2023-09-11/01-07/'+name+'.txt'

def read_data(filepath):
    with open(filepath, "r") as f:
        lines = f.readlines()
    data = [float(line.split()[-3]) for line in lines]
    return data

data1 = read_data(filepath1)
data2 = read_data(filepath2)

plt.hist(data1, bins='auto', edgecolor='k', color='cyan', alpha=0.5, label='After Filter')
plt.hist(data2, bins='auto', edgecolor='k', color='magenta', alpha=0.5, label='Origin')

plt.xlabel("Value")
plt.ylabel("Frequency")
plt.title("Score Distribution Differences")
plt.legend()
plt.show()