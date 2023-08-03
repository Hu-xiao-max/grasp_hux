import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull
from scipy.optimize import minimize
from grasp_quality_measurement_functions import get_friction_cone_vector, compute_force_convex_hull_function



def plot_precision_grasp9():
    '''
    contact model has to be soft finger model
    '''
    contact_p1 = np.array([[0, 0.001, 0]]).T
    contact_p2 = np.array([[0, -0.001, 0]]).T
    contact_n1 = np.array([0, -1, 0])
    contact_n2 = np.array([0, 1, 0])

    friction_cone1 = get_friction_cone_vector(contact_n1, 0.8)
    friction_cone2 = get_friction_cone_vector(contact_n2, 0.8)

    rho = 0.01

    primitive_torque1 = np.cross(contact_p1, friction_cone1, axisa = 0, axisb = 0)
    primitive_torque2 = np.cross(contact_p2, friction_cone2, axisa = 0, axisb = 0)

    wrench1 = np.hstack([friction_cone1.T, primitive_torque1 / rho])
    wrench2 = np.hstack([friction_cone2.T, primitive_torque2 / rho])

    wrenches = np.vstack([wrench1, wrench2])

    print("friction cone 1:")
    print(friction_cone1)
    print("friction cone 2:")
    print(friction_cone2)
    print("wrenches")
    print(wrenches)
    #print(compute_force_convex_hull_function(np.vstack([friction_cone1.T, friction_cone2.T]), draw = True))

    print(ConvexHull(wrenches))


    print(compute_force_convex_hull_function(wrenches))


def draw_optimal_contact_grasp8(q):
    theta_range = np.arange(0, 2 * np.pi, 0.05)
    height_range = np.arange(-0.03, 0.03, 0.002)
    sample = np.meshgrid(theta_range, height_range)
    print(sample[1].flatten().shape)
    contact_p_set = np.vstack([0.005 * np.cos(sample[0].flatten()), 0.005 * np.sin(sample[0].flatten()), sample[1].flatten()])

    best_contact_p = np.array([[0.005, 0.005 * np.cos(q[0]), 0.005 * np.cos(q[2])],
                                [0, 0.005 * np.sin(q[0]), 0.005 * np.sin(q[2])],
                                [0, q[1], q[3]]])
    
    
    fig = plt.figure()
    ax1 = plt.axes(projection='3d')

    ax1.scatter(contact_p_set[0], contact_p_set[1], contact_p_set[2], linewidths=1, color = 'royalblue')
    ax1.scatter(best_contact_p[0], best_contact_p[1], best_contact_p[2], color = 'red', linewidths=10)
    plt.show()

def find_optimal_contact_grasp8_optimization():
    def objective(q):
        contact_p_set = np.array([[0.005, 0.005 * np.cos(q[0]), 0.005 * np.cos(q[2])],
                                  [0, 0.005 * np.sin(q[0]), 0.005 * np.sin(q[2])],
                                  [0, q[1], q[3]]])
        contact_n_set = np.array([[-1, -np.cos(q[0]), -np.cos(q[2])],
                                  [0, -np.sin(q[0]), -np.sin(q[2])],
                                  [0, 0, 0]])
        mrw = precision_grasp8(contact_p_set, contact_n_set)

        return -mrw
    
    res = minimize(objective, np.array([np.pi, 0.001, np.pi, -0.001]), method = "SLSQP", 
                   bounds = ((np.pi / 2, 3 * np.pi / 2), (-0.03, 0.03), (np.pi / 2, 3 * np.pi / 2), (-0.03, 0.03)))
    
    print(res)

    draw_optimal_contact_grasp8(res['x'])

def precision_grasp8(contact_p_set, contact_n_set):
    """
    Args:
    - contact_p_set (3, n): vertical vector of (contact_p1, contact_p2, contact_p3)
    - contact_n_set (3, n): vertical vector of (contact_n1, contact_n2, contact_n3)
    """

    contact_p1 = contact_p_set[:, :1]
    contact_p2 = contact_p_set[:, 1:2]
    contact_p3 = contact_p_set[:, 2:3]
    contact_n1 = contact_n_set[:, 0]
    contact_n2 = contact_n_set[:, 1]
    contact_n3 = contact_n_set[:, 2]

    friction_cone1 = get_friction_cone_vector(contact_n1, 0.8)
    friction_cone2 = get_friction_cone_vector(contact_n2, 0.8)
    friction_cone3 = get_friction_cone_vector(contact_n3, 0.8)

    rho = 0.1

    primitive_torque1 = np.cross(contact_p1, friction_cone1, axisa = 0, axisb = 0)
    primitive_torque2 = np.cross(contact_p2, friction_cone2, axisa = 0, axisb = 0)
    primitive_torque3 = np.cross(contact_p3, friction_cone3, axisa = 0, axisb = 0)

    wrench1 = np.hstack([friction_cone1.T, primitive_torque1 / rho])
    wrench2 = np.hstack([friction_cone2.T, primitive_torque2 / rho])
    wrench3 = np.hstack([friction_cone3.T, primitive_torque3 / rho])

    wrenches = np.vstack([wrench1, wrench2, wrench3])

    # print("friction cone 1:")
    # print(friction_cone1)
    # print("friction cone 2:")
    # print(friction_cone2)
    # print("wrenches")
    # print(wrenches)
    #print(compute_force_convex_hull_function(np.vstack([friction_cone1.T, friction_cone2.T]), draw = True))


    return compute_force_convex_hull_function(wrenches)

def plot_angle_error_lwr():
    ang_list = []
    mrw_list = []
    for ang in np.arange(0, np.pi / 4, 0.01):
        contact_p_set = np.array([[0.005 * np.cos(ang), 0.005 * np.cos(np.pi - ang), 0.005 * np.cos(np.pi - ang)],
                                [0.005 * np.sin(ang), 0.005 * np.sin(np.pi - ang), 0.005 * np.sin(np.pi - ang)],
                                [0, 0.03, -0.03]])
        contact_n_set = np.array([[-np.cos(ang), -np.cos(np.pi - ang), -np.cos(np.pi - ang)],
                                    [-np.sin(ang), -np.sin(np.pi - ang), -np.sin(np.pi - ang)],
                                    [0, 0, 0]])
        
        mrw = precision_grasp8(contact_p_set, contact_n_set)
        if mrw > 0:
            ang_list.append(ang)
            mrw_list.append(mrw)
    
    fig = plt.figure()
    plt.plot(np.array(ang_list) / np.pi * 180, mrw_list)
    plt.xlabel("Norm vector error angle/degrees")
    plt.ylabel("Minimum resisted wrench magnitude")

    plt.savefig("error_angle_mrw.png")

    plt.show()

if __name__ == "__main__":
    # find_optimal_contact_grasp8_optimization()
    plot_angle_error_lwr()