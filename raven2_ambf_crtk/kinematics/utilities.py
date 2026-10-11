import numpy as np

def quaternion_to_rotation_matrix(x, y, z, w):
    R = np.array([
        [1 - 2*(y**2 + z**2),  2*(x*y - w*z),  2*(x*z + w*y)],
        [2*(x*y + w*z),  1 - 2*(x**2 + z**2),  2*(y*z - w*x)],
        [2*(x*z - w*y),  2*(y*z + w*x),  1 - 2*(x**2 + y**2)]
    ])
    return R

def get_Origin(T_matrix):
    origin = T_matrix[:3,3]
    return origin

def get_Basis(T_matrix):
    basis = np.zeros((3,3))
    basis = T_matrix[0:3,0:3]
    return basis