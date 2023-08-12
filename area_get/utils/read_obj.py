import numpy as np
def read_obj_vertices(file_path):
    vertices = []

    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('v '):
                vertex = line.strip().split(' ')[1:]
                vertex = [float(coord) for coord in vertex]
                vertices.append(vertex)

    return vertices


if __name__=='__main__':
    obj_file_path = './object/nontextured.obj'
    vertices = read_obj_vertices(obj_file_path)
    print(len(vertices))
    vertices=np.array(vertices)

    print(vertices.shape)

    # list=[]

    # for vertex in vertices:
    #     list.append(vertex)
    # print(list)