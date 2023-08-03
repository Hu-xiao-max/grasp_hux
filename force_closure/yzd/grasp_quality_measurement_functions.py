import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from scipy.linalg import null_space
import time


def get_friction_cone_vector(normal_vector, friction_coefficient):
    '''
    :param normal_vector:[a,b,c] or (a,b,c) or np.array([a,b,c]),normal_vector is must be transformed to a unit vector
    :param friction_coefficient:
    :return: friction_vector 3*4 dimensions numpy.ndarray. it is combined by np.hstack((vector_1,2,3,4)),vector_1 is a 3x1 vector

    This function is used for computing friction cone vector, which can be represented as the friction cone.

    normal_vector is [a,b,c], the plane vertical to [a,b,c] is ax+by+cz=0, we find 4 vectors that divide the plane into four parts
    so plane_vector_1 is [c,0,-a], plane_vector_2 is [-c,0,a], plane_vector_3 is [ab,-a^2-c^2,bc],
    plane_vector_4 is [-ab,a^2+c^2,-bc], and then we unitize these four vectors
    if axb,the cross product matrix a is [[0,-a3,a2]
                                          [a3,0,-a1]
                                          [-a2,a1,0]],we represent this matrix is ^a, so axb=np.dot(^a,b)
    we need to find out the friction_vector which is vertical outward to friction cone, so there are two equations must be required:
    1.np.dot(normal_vector,friction_vector)=cos(pi/2-theta)  assume that normal_vector and friction vector are both unit vectors
    2.np.cross(normal_vector,friction_vector)=sin(pi/2-theta)*plane_vector
    so  np.dot(np.vstack((normal_vector,^normal_vector)),friction_vector] = np.vstack((cos(pi/2-theta),sin(pi/2-theta)*plane_vector)),
        note plane_vector is 3X1 vector,normal_vector is 1X3 vector

    Note: this normal vector is towards to link, frcition_vector is towards to the link, and the component of the friction vector
          which is normal to the link must be 1.
    '''
    theta = math.atan(friction_coefficient)
    normal_vector = np.array(normal_vector)
    # todo:this equation is not necessary,normal_vector is initially unit vector
    normal_vector = normal_vector / np.linalg.norm(normal_vector)

    plane_vector_1 = np.array([[normal_vector[2], 0, -normal_vector[0]]]).T
    plane_vector_3 = np.array([[normal_vector[0] * normal_vector[1],
                                -normal_vector[0] * normal_vector[0] - normal_vector[2] * normal_vector[2],
                                normal_vector[1] * normal_vector[2]]]).T
    
    if normal_vector[0] == 0 and normal_vector[2] == 0:
        plane_vector_1 = np.array([[1, 0, 0]]).T
        plane_vector_3 = np.array([[0, 0, 1]]).T

    plane_vector_1 = plane_vector_1 / np.linalg.norm(plane_vector_1)
    plane_vector_2 = -plane_vector_1
    plane_vector_3 = plane_vector_3 / np.linalg.norm(plane_vector_3)
    plane_vector_4 = -plane_vector_3
    normal_vector_cross_product_matrix = np.array([[0, -normal_vector[2], normal_vector[1]],
                                                   [normal_vector[2], 0, -normal_vector[0]],
                                                   [-normal_vector[1], normal_vector[0], 0],
                                                   ])

    normal_vector_cross_product_matrix_add_normal_vector = np.vstack(
        (normal_vector, normal_vector_cross_product_matrix))


    plane_vector_1_add_costheta = np.vstack(
        (np.array([math.cos(theta)]), math.sin(theta) * plane_vector_1))
    plane_vector_2_add_costheta = np.vstack(
        (np.array([math.cos(theta)]), math.sin(theta) * plane_vector_2))
    plane_vector_3_add_costheta = np.vstack(
        (np.array([math.cos(theta)]), math.sin(theta) * plane_vector_3))
    plane_vector_4_add_costheta = np.vstack(
        (np.array([math.cos(theta)]), math.sin(theta) * plane_vector_4))
    plane_vector_add_costheta = np.hstack((plane_vector_1_add_costheta, plane_vector_2_add_costheta,
                                           plane_vector_3_add_costheta, plane_vector_4_add_costheta))



    friction_vector = np.dot(np.linalg.pinv(normal_vector_cross_product_matrix_add_normal_vector),
                             plane_vector_add_costheta)


    #todo:Note that the component of the friction vector which is normal to the link should be 1.
    return friction_vector/math.cos(theta)


def compute_force_convex_hull_function(points, draw = False):
    '''
    :param points: dimension is n*3, the type should be numpy array finally
    :return: reward: this is convex hull minimum distance

    This function is used to compute the minimum distance of the force convex hull.
    The function logic is:
    1.compute the vertices of points (ConvexHull)
    2.get all superplane (ConvexHull.simplices record all vertice indexes)
    3.get all superplane vertical vector
    4.let all directions of vertical vectors outward
    5.compute the distance from original point to any superplane
    6.if any distance <=0, the grasp is not stable

    '''
    try:
        points=np.array(points)
        hull = ConvexHull(points)
        core = np.sum(points, axis=0) / len(points)

        if draw:
            fig = plt.figure()
            ax1 = plt.axes(projection='3d')
            ax1.plot3D(points[:,0], points[:,1], points[:,2],'o',c='g')
            ax1.plot3D(core[0],core[1],core[2],'o',lw=6,c='r')

        count = 0
        flags = 1
        distance_list=[]
        for simplex in hull.simplices:
            count = count + 1
            if draw:
                simplex_plot=np.append(simplex,[simplex[0]],axis=0)
                ax1.plot3D(points[simplex_plot, 0], points[simplex_plot, 1],points[simplex_plot, 2], 'k-')
            # todo:superplane_vector is n-1 n-vectors, representing one superplane
            # todo:vertical_vector is vertical vector about this superplane
            superplane_vector = points[simplex] - points[simplex][0]
            vertical_vector = null_space(superplane_vector)
            # todo:core_toward_outward is the direction from the core to edge point, it can adjust vertical_vector direction to outward
            core_toward_outward_vector = points[simplex][0] - core
            vertical_vector = vertical_vector.flatten()
            # todo:because some vertices are two close,the result of null_space may not be 1 n-vector.We should discard bad result.
            if len(vertical_vector)==6 or len(vertical_vector) == 3:
                if np.dot(vertical_vector, core_toward_outward_vector) < 0:
                    vertical_vector = -vertical_vector
                # vertical_list.append(vertical_vector)

                original_toward_superplane = points[simplex][0]
                distance = np.dot(original_toward_superplane, vertical_vector)
                # print ('grasp distance is {0}'.format(distance))
                if distance <= 0:
                    # print('grasp is not stable')
                    flags = 0
                    break
                else:
                    distance_list.append(distance)
        
        if draw:
            plt.show()
        if flags==0:
            # print ('grasp is not stable')
            return -1
        elif flags==1:
            reward=min(distance_list)
            # print ('compute hull function time is {0}'.format(time.time()-start_time))
            # print ('the force convex hull min distance is {0}'.format(reward))
            return reward
    except:
        # print ('grasp is not solvable')
        return -1


def get_friction_cone_vertical_inwards_vector(normal_vector, friction_coefficient):
    '''
    :param normal_vector:[a,b,c] or (a,b,c) or np.array([a,b,c]),normal_vector is must be transformed to a unit vector
    :param friction_coefficient:
    :return: friction_vector 3*4 dimensions numpy.ndarray. it is combined by np.hstack((vector_1,2,3,4)),vector_1 is a 3x1 vector

    This function is used for computing the vector which is normal to friction cone vector inwards.
    If contact force is within the friction cone, np.dot(friction_vector.T,force)<=0.

    normal_vector is [a,b,c], the plane vertical to [a,b,c] is ax+by+cz=0, we find 4 vectors that divide the plane into four parts
    so plane_vector_1 is [c,0,-a], plane_vector_2 is [-c,0,a], plane_vector_3 is [ab,-a^2-c^2,bc],
    plane_vector_4 is [-ab,a^2+c^2,-bc], and then we unitize these four vectors
    if axb,the cross product matrix a is [[0,-a3,a2]
                                          [a3,0,-a1]
                                          [-a2,a1,0]],we represent this matrix is ^a, so axb=np.dot(^a,b)
    we need to find out the friction_vector which is vertical outward to friction cone, so there are two equations must be required:
    1.np.dot(normal_vector,friction_vector)=cos(pi/2+theta)  assume that normal_vector and friction vector are both unit vectors
    2.np.cross(normal_vector,friction_vector)=sin(pi/2-theta)*plane_vector
    so  np.dot(np.vstack((normal_vector,^normal_vector)),friction_vector] = np.vstack((cos(pi/2+theta),sin(pi/2+theta)*plane_vector)),
        note plane_vector is 3X1 vector

    Note: this normal vector is towards to link, frcition_vector is normal to friction cone inwards.
          Attension, this friction_vector is not the edge of friction cone!!!
    '''
    theta = math.atan(friction_coefficient)
    normal_vector = np.array(normal_vector)
    # todo:this equation is not necessary,normal_vector is initially unit vector
    normal_vector = normal_vector / np.linalg.norm(normal_vector)

    plane_vector_1 = np.array([[normal_vector[2], 0, -normal_vector[0]]]).T
    plane_vector_3 = np.array([[normal_vector[0] * normal_vector[1],
                                -normal_vector[0] * normal_vector[0] - normal_vector[2] * normal_vector[2],
                                normal_vector[1] * normal_vector[2]]]).T

    plane_vector_1 = plane_vector_1 / np.linalg.norm(plane_vector_1)
    plane_vector_2 = -plane_vector_1
    plane_vector_3 = plane_vector_3 / np.linalg.norm(plane_vector_3)
    plane_vector_4 = -plane_vector_3
    normal_vector_cross_product_matrix = np.array([[0, -normal_vector[2], normal_vector[1]],
                                                   [normal_vector[2], 0, -normal_vector[0]],
                                                   [-normal_vector[1], normal_vector[0], 0],
                                                   ])

    # print('normal_vector is {0}'.format(normal_vector))
    normal_vector_cross_product_matrix_add_normal_vector = np.vstack(
        (normal_vector, normal_vector_cross_product_matrix))

    # print('normal_vector_cross_product_matrix_add_normal_vector is {0}'.format(
    #     normal_vector_cross_product_matrix_add_normal_vector))

    plane_vector_1_add_costheta = np.vstack(
        (np.array([math.cos(math.pi / 2 + theta)]), math.sin(math.pi / 2 + theta) * plane_vector_1))
    plane_vector_2_add_costheta = np.vstack(
        (np.array([math.cos(math.pi / 2 + theta)]), math.sin(math.pi / 2 + theta) * plane_vector_2))
    plane_vector_3_add_costheta = np.vstack(
        (np.array([math.cos(math.pi / 2 + theta)]), math.sin(math.pi / 2 + theta) * plane_vector_3))
    plane_vector_4_add_costheta = np.vstack(
        (np.array([math.cos(math.pi / 2 + theta)]), math.sin(math.pi / 2 + theta) * plane_vector_4))
    plane_vector_add_costheta = np.hstack((plane_vector_1_add_costheta, plane_vector_2_add_costheta,
                                           plane_vector_3_add_costheta, plane_vector_4_add_costheta))

    # print('plane_vector_add_costheta is {0}'.format(plane_vector_add_costheta))

    print("function2: ")
    print(plane_vector_add_costheta)
    friction_vector = np.dot(np.linalg.pinv(normal_vector_cross_product_matrix_add_normal_vector),
                             plane_vector_add_costheta)

    # print('firction_vector is {0}'.format(friction_vector))

    # todo: in order to let np.dot(friction_vector.T,force)<=0, because we use cvxopt library to do quadratic optimization,
    return friction_vector


def get_finger_friction_cone_jocobian_grasp_matrix(contact_position, contact_normal_vector, joint_position,
                                                   joint_orientation,
                                                   object_position, friction_coefficient=1):
    '''
    :param contact_position: [link_number,],if there are 3 links, it is like [[(),()],[],[]], () is 1X3 contact position
    :param contact_normal_vector: [link_number,], its dimension is the same as contact position,
           normal direction is towards robotic hand and vertical to the object surface
    :param joint_position: [1,link_number],if there are 3 links, it is like [(),(),()],() is 1X3 joint position
    :param joint_orientation: [1,link_number], its dimension is the same as joint position
    :param object_position: 1*3 object position
    :param friction_coefficient: it depends on material properties
    :return:N,J,G,link_number,contact_number: N is (4*contact_number,3*contact_number), J is (link_number,contact_number*3)
                                              G is (6,contact_number*3)

    In order to evaluate hand configuration quality, we use energy minimization as the objective function.
    When robotic hand stably grasps the object, motor is locked-rotor. Energy is dissipated in the form of heat.
    So E is proportional to I^2*R
    Assuming R cannot increase with increasing temperature, so E ~ I^2
    Because joint torque is proportional to motor electric current, E ~ I^2 ~ torque^2
    The target optimization can be shown as:
    min sum(torque_i^2)
    s.t. Gf_i = -W_g (G is grasp matrix, W_g is gravity wrench, this equation indicates all f must balance object gravity)
         N_i.T dot f_i <=0 (all f must be within friction cone)
         J_i dot f_i = torque_i <= maximal motor torque_i (all joint torque must be lower than the maximum torque of the motor

    This function is aim to get N, J and G by one finger

    Assuming there are 3 joints and 4 contact points, the connect sequence is link2-joint2-link1-joint1-link0-joint0-base
    N is like [N_0.T   0     0     0
                0    N_1.T   0     0
                0      0   N_2.T   0
                0      0     0   N_3.T], because N_i is a 3*4 matrix, N.T is 4n*3n
    f is like [f_0
               f_1
               f_2
               f_3], because f_i is 3*1 vector, f is 3n*1

    J is like [J_00, J_01, J_02, J_03
               J_10, J_11, J_12, J_13
               J_20, J_21, J_22, J_23], J dimension is joint_number * (contact_number*3)
                                        J_ij is jth contact position relative to ith joint jocobian matrix
                                        torque_i = [J_i0,J_i1,J_i2,J_i3] dot f
                                        joint_orientation, contact_position and joint_position are all 1X3 vectors
                                        torque_ij = joint_orientation_i dot ^(contact_position_j-joint_position_i) dot f_j
                                        J_ij = joint_orientation_i dot ^(contact_position_j-joint_position_i)
                                        J_ij is 1*3 vectors

    Note: We must know which link jth contact points is in. We do not need to compute J_ij if jth contact is in <ith link.
          For example, if point 0 is in the link0, we need to compute J_00, but do not need to compute J_10 and J_20

    G is grasp matrix, -[  I,    I,    I,    I
                         ^p_0, ^p_1, ^p_2, ^p_3], ^p_i is cross product matrix of (contact_position_i-object_position),
                         I is 3*3 unit matrix
                         ^p_i is 3*3 matrix
                         Gf=-W_g, because this f is towards to robotic hand, so we need to add minus in the G
    '''
    link_number = len(contact_position)
    contact_number = 0
    for i in range(link_number):
        if len(contact_position[i]) != 0:
            contact_number = contact_number + len(contact_position[i])
    contact_normal_vector_flatten = [i for k in contact_normal_vector for i in k]

    N = np.zeros([4 * contact_number, 3 * contact_number])
    for i in range(contact_number):
        N[i * 4:i * 4 + 4, i * 3:i * 3 + 3] = get_friction_cone_vertical_inwards_vector(
            contact_normal_vector_flatten[i], friction_coefficient).T

    J = np.zeros([link_number, contact_number * 3])
    point_index = -1
    for i in range(link_number):
        link_point_number = len(contact_position[i])
        if link_point_number != 0:
            for j in range(link_point_number):
                point_index = point_index + 1
                for k in range(0, i + 1):
                    contact_plus_joint_position = np.array(contact_position[i][j]) - np.array(joint_position[k])
                    contact_plus_joint_position_cross_product_matrix = \
                        np.array([[0, -contact_plus_joint_position[2], contact_plus_joint_position[1]],
                                  [contact_plus_joint_position[2], 0, -contact_plus_joint_position[0]],
                                  [-contact_plus_joint_position[1], contact_plus_joint_position[0], 0]])
                    J[k, point_index * 3:point_index * 3 + 3] = np.dot(joint_orientation[k],
                                                                       contact_plus_joint_position_cross_product_matrix)

    G = np.zeros([6, contact_number * 3])
    contact_position_flatten = [i for k in contact_position for i in k]
    for i in range(contact_number):
        G[0:3, i * 3:i * 3 + 3] = np.eye(3)
        contact_plus_object_position = np.array(contact_position_flatten[i]) - np.array(object_position)
        G[3:6, i * 3:i * 3 + 3] = np.array([[0, -contact_plus_object_position[2], contact_plus_object_position[1]],
                                            [contact_plus_object_position[2], 0, -contact_plus_object_position[0]],
                                            [-contact_plus_object_position[1], contact_plus_object_position[0], 0]])

    # todo:Attension, because the fi direction is toward to the robotic hand, if we use the same fi to compute object wrench, we must
    # todo: add - in fornt of G, so G should be -G
    G = -G

    # print('link number is {0},contact_number is {1}'.format(link_number, contact_number))
    # print('contact_normal_vector_flatten is {0}'.format(contact_normal_vector_flatten))
    # print('N is {0}'.format(N))
    # print('N dimensions are {0}'.format(N.shape))
    # print('J is {0}'.format(J))
    # print('G is {0}'.format(G))

    return N, J, G, link_number, contact_number


def integrate_all_finger_jocobian_cone_grasp_matrix(contact_position_all,contact_normal_vector_all,joint_position_all,joint_orientation_all,
                                                    object_position,friction_coefficient=1):
    '''
    :param contact_position_all: [contact_position,contact_position.....], shape is [finger_number,link_number,]
    :param contact_normal_vector_all: [contact_normal_vector,contact_normal_vector.....], shape is [finger_number,link_number,]
    :param joint_position_all: shape is [finger_number,link_number]
    :param joint_orientation_all: shape is [finger_number,link_number]
    :param object_position:
    :param friction_coefficient:
    :return: N,J,G,link_number_sum,contact_number_sum. N is (4*contact_number_sum,3*contact_number_sum),
                                                       J is (link_number_sum,contact_number_sum*3)
                                                       G is (6,contact_number_sum*3)
    This function is integrate N,J,G of each finger,assuming there are two fingers
    N=[N0  0
       0  N1]
    J=[J0 0
       0  J1]
    G=[G0,G1]
    '''
    finger_number=len(contact_position_all)
    N_all=[]
    J_all=[]
    G_all=[]
    link_number_all=[]
    contact_number_all=[]
    for i in range(finger_number):
        N,J,G,link_number,contact_number=\
            get_finger_friction_cone_jocobian_grasp_matrix(contact_position_all[i],contact_normal_vector_all[i],
                                                           joint_position_all[i],joint_orientation_all[i],object_position,
                                                           friction_coefficient)
        if contact_number!=0:
            N_all.append(N)
            J_all.append(J)
            G_all.append(G)
            link_number_all.append(link_number)
            contact_number_all.append(contact_number)

    link_number_sum=sum(link_number_all)
    contact_number_sum=sum(contact_number_all)

    valid_N_number=len(N_all)
    N=np.zeros([4*contact_number_sum,3*contact_number_sum])
    J=np.zeros([link_number_sum,3*contact_number_sum])
    G=np.zeros([6,3*contact_number_sum])
    link_current_index=0
    contact_current_index=0
    for i in range(valid_N_number):
        N[4*contact_current_index:4*(contact_current_index+contact_number_all[i]),
                                     3*contact_current_index:3*(contact_current_index+contact_number_all[i])]=N_all[i]
        J[link_current_index:link_current_index+link_number_all[i],
                                     3*contact_current_index:3*(contact_current_index+contact_number_all[i])]=J_all[i]
        G[0:6,3*contact_current_index:3*(contact_current_index+contact_number_all[i])]=G_all[i]
        link_current_index=link_current_index+link_number_all[i]
        contact_current_index=contact_current_index+contact_number_all[i]


    #todo:now,link_current_index=link_number_sum,contact_current_index=contact_number_sum
    # print ('total N,J,G is {0},{1},{2},{3},{4}'.format(N,J,G,link_current_index,contact_current_index))
    # print ('N is {0}'.format(N))
    # print ('J is {0}'.format(J))
    # print ('G is {0}'.format(G))
    # print ('G type is {0}'.format(type(G)))
    # print ('g shape is {0}'.format(G.shape))
    return N,J,G,link_number_sum,contact_number_sum


def get_minmum_energy_result(N,J,G,contact_number,W_g,joint_torque_limit):

    return
    '''
    :param N: get from integrate_all_finger_jocobian_cone_grasp_matrix
    :param J: get from integrate_all_finger_jocobian_cone_grasp_matrix
    :param G: get from integrate_all_finger_jocobian_cone_grasp_matrix
    :param contact_number: get from integrate_all_finger_jocobian_cone_grasp_matrix,==contact_number_sum
    :param W_g: need (6,1) format np.array
    :param joint_torque_limit: list (1,joint_number), need to transform (joint_number,1) format np.array
    :return:

    This function is get energy minmum
    min sum(norm(Jf,2))
    s.t. Gf_i = -W_g (G is grasp matrix, W_g is gravity wrench, this equation indicates all f must balance object gravity)
         N dot f_i <=0 (all f must be within friction cone)
         J_i dot f_i = torque_i <= abs(maximal motor torque_i) (all joint torque must be lower than the maximum torque of the motor
    '''
    # print ('solve energy reward')
    N=np.array(N)
    J=np.array(J)
    G=np.array(G)
    W_g=np.array(W_g).reshape(-1,1)
    joint_torque_limit=np.array(joint_torque_limit).reshape(-1,1)
    f = cvx.Variable((contact_number*3, 1))
    # print ('f type is {0},shape is {1}'.format(type(f),f.shape))
    objective=cvx.Minimize(cvx.sum_squares(J@f))
    # objective = cvx.Minimize(cvx.norm(J @ f, 2))
    constraints = [G @ f == -W_g, N @ f <= np.zeros((contact_number*4,1)), J@f <= abs(joint_torque_limit),J @ f >= -abs(joint_torque_limit)]
    prob = cvx.Problem(objective, constraints)
    try:
        prob.solve()
        if prob.status == 'optimal':
            # print ('有解')
            # print ('f is {0}'.format(f.value))
            # print ('Jf is {0}'.format(np.dot(J,f.value)))
            # print ('optimal value is {0}'.format(prob.value))
            return prob.value
        else:
            # print ('无解')
            print('energy 解不好')
            return -1
    except:
        print('energy 无解')
        return -1

def get_minmum_energy_result_plus_palm(N, J, G, contact_number, W_g, joint_torque_limit,palm_contact_number,N_palm,G_palm):

    return
    '''
    :param N: get from integrate_all_finger_jocobian_cone_grasp_matrix
    :param J: get from integrate_all_finger_jocobian_cone_grasp_matrix
    :param G: get from integrate_all_finger_jocobian_cone_grasp_matrix
    :param contact_number: get from integrate_all_finger_jocobian_cone_grasp_matrix,==contact_number_sum
    :param W_g: need (6,1) format np.array
    :param joint_torque_limit: list (1,joint_number), need to transform (joint_number,1) format np.array
    :return:

    This function is get energy minmum
    min sum(norm(Jf,2))
    s.t. Gf_i = -W_g (G is grasp matrix, W_g is gravity wrench, this equation indicates all f must balance object gravity)
         N dot f_i <=0 (all f must be within friction cone)
         J_i dot f_i = torque_i <= abs(maximal motor torque_i) (all joint torque must be lower than the maximum torque of the motor
    '''
    # print('solve energy reward plus palm')
    N = np.array(N)
    J = np.array(J)
    G = np.array(G)
    N_palm=np.array(N_palm)
    G_palm=np.array(G_palm)
    W_g = np.array(W_g).reshape(-1, 1)
    joint_torque_limit = np.array(joint_torque_limit).reshape(-1, 1)
    f = cvx.Variable((contact_number * 3, 1))
    f_palm=cvx.Variable((palm_contact_number*3,1))
    # print('f type is {0},shape is {1}'.format(type(f), f.shape))
    objective = cvx.Minimize(cvx.sum_squares(J @ f))
    # objective = cvx.Minimize(cvx.norm(J @ f, 2))
    constraints = [G @ f + G_palm @ f_palm== -W_g, N @ f <= np.zeros((contact_number * 4, 1)),
                   N_palm @ f_palm <= np.zeros((palm_contact_number*4,1)),J @ f <= abs(joint_torque_limit),J @ f >= -abs(joint_torque_limit)]
    prob = cvx.Problem(objective, constraints)
    try:
        prob.solve()
        if prob.status == 'optimal':
            # print ('有解')
            # print ('f is {0}'.format(f.value))
            # print ('Jf is {0}'.format(np.dot(J,f.value)))
            # print ('optimal value is {0}'.format(prob.value))
            return prob.value
        else:
            print('energy 解不好')
            return -1
    except:
        print ('energy 无解')
        return -1



# if __name__ == "__main__":
#   points=2*np.random.rand(200, 3)-1
#   compute_force_convex_hull_function(points)

