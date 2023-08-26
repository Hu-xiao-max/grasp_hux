
import numpy as np

vector1 = np.array([1, 2, 3])
vector2 = np.array([4, 5, 6])
vector3 = np.array([7, 8, 9])

matrix = np.column_stack((vector1, vector2, vector3))

print(matrix)