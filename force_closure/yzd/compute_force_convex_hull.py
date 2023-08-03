import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from scipy.linalg import null_space

'''
找出凸包最短边的算法流程如下：
1.index=quickhull(inputs) 第一步是要先找出凸包边缘点的索引，一般有现成的算法包，这里的quickhull是代指，在本程序中具体指的是函数Convexhull
2.core=inputs.sum()/input.size() 找到凸包内的一点
3.根据index求出每个面的法向量，具体求法就是每两个点组成一条边，将一个点与其他n-1条组成n-1个向量，去求解n×（n-1）维矩阵的零向量空间，求得的零向量即为法向量
4.根据面上任一点与core组成的向量 点乘 上述的法向量，若值为正，说明法向量方向朝凸包外，若为负，说明方向朝内，这时修改法向量方向
5.判断原点是否在凸包内，即判断（原点到所有超平面任意点的向量点乘超平面对应的单位法向量）是否大于0，若其中有负数,说明抓取不稳定，若没有负数，返回最短边的值，即为所求的衡量抓取稳定性的值。
'''

#创建三维坐标系
fig = plt.figure()
ax1 = plt.axes(projection='3d')

#随机生成200个三维点
points = 2*np.random.rand(200, 3)-1   # 200 random points in 3-D
#得到这200个组成的凸包最外边的点的集合
hull = ConvexHull(points)

#画出200个点在三维坐标系下的位置，绿色点表示
ax1.plot3D(points[:,0], points[:,1], points[:,2],'o',c='g')

#200个点累加的平均值所代表的点一定在凸包内
core=np.sum(points,axis=0)/len(points)

#画出core这个在凸包内的点，红色点表示
ax1.plot3D(core[0],core[1],core[2],'o',lw=6,c='r')


count=0
distance_list=[]
vertical_list=[]
flags=1
#hull.simplices表示凸包每个边缘面上的三个点在整个凸包点集合里的索引
#todo:计算每个边缘面与原点的距离，若距离为负，说明零点不在凸包内
for simplex in hull.simplices:
    count=count+1
    #todo: draw edge lines
    simplex_plot=np.append(simplex,[simplex[0]],axis=0)
    print ('simlex is {0}'.format(simplex))
    print ('simplex_plot is {0}'.format(simplex_plot))
    ax1.plot3D(points[simplex_plot, 0], points[simplex_plot, 1],points[simplex_plot, 2], 'k-')
    #todo:superplane_vector is n-1 n-vectors, representing one superplane
    #todo:vertical_vector is vertical vector about this superplane
    superplane_vector=points[simplex]-points[simplex][0]
    print ('points[simplex] is {0}'.format(points[simplex]))
    print ('superplane_vector is {0}'.format(superplane_vector))
    # A = np.array(superplane_vector)
    A=superplane_vector
    vertical_vector = null_space(A)
    print ('vertical_vector is {0}'.format(vertical_vector))
    #todo:core_toward_outward is the direction from the core to edge point, it can adjust vertical_vector direction to outward
    core_toward_outward_vector=points[simplex][0]-core
    print ('points[simplex] is {0}'.format(points[simplex]))
    vertical_vector=vertical_vector.flatten()
    if np.dot(vertical_vector,core_toward_outward_vector)<0:
        vertical_vector=-vertical_vector
    vertical_list.append(vertical_vector)
    print ('count is {0}'.format(count))
    print (np.dot(vertical_vector,core_toward_outward_vector))
    original_toward_superplane=points[simplex][0]
    distance=np.dot(original_toward_superplane,vertical_vector)
    if distance<=0:
        print ('grasp is not stable')
        flags=0
        break
    else:
        distance_list.append(distance)
print ('distance list is {0}'.format(distance_list))
distance_min_index=distance_list.index(min(distance_list))
min_distance_toward_point=distance_list[distance_min_index]*vertical_list[distance_min_index]
ax1.plot3D([0,min_distance_toward_point[0]],[0,min_distance_toward_point[1]],[0,min_distance_toward_point[2]],'k-',lw=6,c='b')
print ('min_distance is {0}'.format(distance_list[distance_min_index]))



plt.show()