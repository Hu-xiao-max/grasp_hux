
import os
datasetpath = '/root/HUX/grasp_hux/area_get/output/2023-09-02/21-10'
obj_list = os.listdir(datasetpath)
for obj in obj_list:
    obj = obj.split('.')[0]
    print(obj)

