
# list=[1,2,1,2,1,6,1,8,2,10]
# list_backup=list.copy()
# index_list=[]
# for t in range(len(list)):
#     index_list.append(t)

# for i in range(len(list)):
#     count=0

#     for j in range(i+1,len(list)):
#         if list[i]==list[j]:
#             list_backup.pop(j)
#             count+=1
#     if count>=1:
#         print(list[i],count)
#         list_backup=list.copy()
   

list=[1,2,1,2,1,6,1,8,2,10]
for i in range(len(list)):
    index=[]
    for j in range(i+1,len(list)):
        if list[i]==list[j]:
            index.append(j)
    if len(index)>= 1:
        for t in index:
            list.pop(t)

