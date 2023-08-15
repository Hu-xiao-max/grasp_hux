import sys
import time
import datetime
import os

# 获取当前时间
now = datetime.datetime.now()
# 将当前时间格式化为字符串
formatted_time = now.strftime('%Y-%m-%d_%H-%M-%S')
if not os.path.exists('./output/'+now.strftime('%Y-%m-%d')):
    os.makedirs('./output/'+now.strftime('%Y-%m-%d'))
f=open('./output/'+now.strftime('%Y-%m-%d')+'/'+now.strftime('%H-%M-%S') +".txt","w")
sys.stdout=f
for i in range(1000):
    print(i)
    print('i+1',i+1)