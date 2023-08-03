import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import pybullet as p
import numpy as np
import math
import pybullet_data_zhong
import random
import time

from urdf_constructed_function import create_hand
from grasp_quality_measurement_functions import get_friction_cone_vector,compute_force_convex_hull_function, \
    get_friction_cone_vertical_inwards_vector,get_finger_friction_cone_jocobian_grasp_matrix,\
    integrate_all_finger_jocobian_cone_grasp_matrix,get_minmum_energy_result


class Self_reconstructed_hand:

    def __init__(self, urdfRootPath=pybullet_data_zhong.getDataPath(), timeStep=1.0/240.0, hand_position=[0,0,0.5], hand_oritation=[0,0,0,1]):
        self.urdfRootPath = urdfRootPath
        self._timeStep = timeStep
        self.maxVelocity = .35
        self.maxForce = 20.
        self.useSimulation = 1
        self.useOrientation = 1
        self.fixed_base = 1
        self.fingertip_index=[]
        self.link_sum=0
        self.hand_position=hand_position
        self.hand_oritation=hand_oritation
        self.finger_number=10
        self.single_finger_joint_number=6
        self.base_xyz=[0.06,0.016,0.05]
        self.link_length_list=[0.016, 0.021, 0.026, 0.031, 0.036, 0.041, 0.046, 0.051, 0.056, 0.062]
        self.link_width=0.008
        self.object_index=2
        self.reset()


    def reset(self):
        # print (self.urdfRootPath)
        self.hand_state=[[0 for col in range(self.single_finger_joint_number*2)] for row in range(self.finger_number)]
        self.hand_state[0]=[1,2,10,1,10,1,10,1,0,0,0,0]
        self.hand_state[4]=[1,2,10,1,10,1,10,1,0,0,0,0]
        self.hand_state[5]=[1,2,10,1,10,1,10,1,0,0,0,0]
        self.hand_state[9]=[1,2,10,1,10,1,10,1,0,0,0,0]
        # print ('self.hand_state is {0}'.format(self.hand_state))

        urdf_file_name,self.finger_abduction_adduction_index,self.finger_abduction_adduction_in_which_side,self.finger_flex_index\
            =create_hand(self.urdfRootPath, self.hand_state, self.base_xyz, self.link_length_list, self.link_width)

        PlaneId=p.createCollisionShape(p.GEOM_PLANE)
        p.createMultiBody(0, PlaneId,-1,[0,0,0])



        self.object_Uid, central_width, central_height=self.choose_object(self.object_index)

        self.self_reconstructed_hand_Uid = p.loadURDF(os.path.join(self.urdfRootPath, urdf_file_name),
                                                      self.hand_position, self.hand_oritation, useFixedBase=1,
                                                      flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)


        self.reset_hand_position_and_orientation(self.object_index, central_width, central_height, math.pi/4, 0.01)



        self.numJoints = p.getNumJoints(self.self_reconstructed_hand_Uid)
        print ('numJoints is {0}'.format(self.numJoints))
        self.jointPositions = [0]* self.numJoints

        p.setGravity(0, 0, -10)
        p.resetDebugVisualizerCamera(0.5, 150, -30, [0, 0, 0])


        self.finger_abduction_adduction_initial_degree=\
            self.choose_inital_hand_abduction_adduction_joint_degree(self.object_index,self.finger_abduction_adduction_index,
                                                                     self.finger_abduction_adduction_in_which_side)

        #todo: set hand friction
        for i in range(self.numJoints):
            p.changeDynamics(self.self_reconstructed_hand_Uid, i, lateralFriction=0.2, spinningFriction=0.001)
        p.changeDynamics(self.self_reconstructed_hand_Uid, -1, lateralFriction=0.2, spinningFriction=0.001)


        for jointIndex in range(self.numJoints):
            p.resetJointState(self.self_reconstructed_hand_Uid, jointIndex, self.jointPositions[jointIndex])
            p.setJointMotorControl2(self.self_reconstructed_hand_Uid,
                                  jointIndex,
                                  p.POSITION_CONTROL,
                                  targetPosition=self.jointPositions[jointIndex],
                                  force=self.maxForce)

        self.power_grasp_planner(0.2,1,20)



    def getActionDimension(self):
        return len(self.motorIndices)



    def choose_object(self,object_index,object_width_or_radius=0.06,object_height=0.12):
        # object_width_or_radius=0.08
        # object_height=0.12
        if object_index==0:
            objectId=p.createCollisionShape(p.GEOM_BOX,halfExtents=[object_width_or_radius, object_width_or_radius, object_height])
            self.object_Uid=p.createMultiBody(0.1, objectId,-1,[0,0,object_height])
            central_width=object_width_or_radius*math.sqrt(2)
            central_height=object_height
        elif object_index==1:
            objectId=p.createCollisionShape(p.GEOM_CYLINDER, radius=object_width_or_radius,height=object_height)
            self.object_Uid = p.createMultiBody(0.1, objectId, -1, [0, 0, object_height/2])
            central_width=object_width_or_radius
            central_height=object_height/2
        elif object_index==2:
            objectId=p.createCollisionShape(p.GEOM_SPHERE, radius=object_width_or_radius)
            self.object_Uid = p.createMultiBody(0.1, objectId, -1, [0, 0, object_width_or_radius])
            central_width = object_width_or_radius
            central_height = object_width_or_radius

        return self.object_Uid, central_width, central_height

    def reset_hand_position_and_orientation(self,object_index,central_width,central_height,theta,offset=0):
        if object_index==0:
            self.hand_oritation = p.getQuaternionFromEuler([math.pi / 2, math.pi / 2, theta])
            self.hand_position=[-(central_width+self.base_xyz[1]/2+offset)*math.cos(theta),-(central_width+self.base_xyz[1]/2+offset)*math.sin(theta),central_height]
            p.resetBasePositionAndOrientation(self.self_reconstructed_hand_Uid, self.hand_position, self.hand_oritation)
        elif object_index==1:
            self.hand_oritation = p.getQuaternionFromEuler([math.pi / 2, math.pi / 2, theta])
            self.hand_position=[-(central_width+self.base_xyz[1]/2+offset)*math.cos(theta),-(central_width+self.base_xyz[1]/2+offset)*math.sin(theta),central_height]
            p.resetBasePositionAndOrientation(self.self_reconstructed_hand_Uid, self.hand_position, self.hand_oritation)
        elif object_index==2:
            self.hand_oritation = p.getQuaternionFromEuler([math.pi / 2, math.pi, theta])
            self.hand_position=[0,0,central_height*2+self.base_xyz[1]/2+offset]
            p.resetBasePositionAndOrientation(self.self_reconstructed_hand_Uid, self.hand_position, self.hand_oritation)

    def choose_inital_hand_abduction_adduction_joint_degree(self,object_index,finger_abduction_adduction_index,finger_abduction_adduction_in_which_side):
        if finger_abduction_adduction_index!=[] and object_index==2:
            joint_list = []
            if finger_abduction_adduction_in_which_side[-1] == -1:
                joint_in_side_1_number = finger_abduction_adduction_in_which_side.index(-1)
            elif finger_abduction_adduction_in_which_side[-1] == 1:
                joint_in_side_1_number = len(finger_abduction_adduction_in_which_side)

            for i in range(len(finger_abduction_adduction_index)):
                joint_list.append(random.randint(-2,2))
            joint_list[0:joint_in_side_1_number]=sorted(joint_list[0:joint_in_side_1_number],reverse=True)
            joint_list[joint_in_side_1_number:len(finger_abduction_adduction_index)]=sorted(joint_list[joint_in_side_1_number:len(finger_abduction_adduction_index)])

            for index in finger_abduction_adduction_index:
                self.jointPositions[index]=joint_list[finger_abduction_adduction_index.index(index)]*math.pi/6
            # print ('joint list is {0}'.format(joint_list))
            return [i*math.pi/6 for i in joint_list]
        elif finger_abduction_adduction_index!=[] and object_index!=2:
            joint_list = [0]*len(finger_abduction_adduction_index)
            return joint_list

        else:
            return []

    def power_grasp_planner(self,friction_coefficient,object_gravity,limit_torque_limit):
        start_time=time.time()
        for i in range(300):
            distance=p.getClosestPoints(self.self_reconstructed_hand_Uid,self.object_Uid,10,len(self.finger_flex_index)
                               +len(self.finger_abduction_adduction_index)-1,-1)[0][8]
            palm_contact_flag=len(p.getContactPoints(self.self_reconstructed_hand_Uid, self.object_Uid, -1, -1))
            if distance>0.001 or palm_contact_flag==0:
                self.apply_finegrs_action_incremental(0.1,self.finger_flex_index)
                self.apply_fingers_action_absolute(self.finger_abduction_adduction_initial_degree,self.finger_abduction_adduction_index)
            else:
                self.apply_finegrs_action_incremental(0.01, self.finger_flex_index)
                self.apply_fingers_action_absolute(self.finger_abduction_adduction_initial_degree, self.finger_abduction_adduction_index)
            p.stepSimulation()
            # time.sleep(self._timeStep)
            
        print ('grasp planner cost time is {0}'.format(time.time()-start_time))
        start_time=time.time()
        force_convex_hull_distance_reward=self.get_force_convex_hull_minimum_distance(friction_coefficient)
        print('compute force convex hull time is {0}'.format(time.time() - start_time))
        energy_reward=self.get_robotic_hand_minimum_energy(self.hand_state,friction_coefficient,object_gravity,limit_torque_limit)

        print('force_convex_hull_distance_reward is {0},energy_reward is {1}'.format(force_convex_hull_distance_reward,
                                                                                     energy_reward))
        # while 1:
        #     p.stepSimulation()
            # time.sleep(self._timeStep)

        return force_convex_hull_distance_reward,energy_reward

    def apply_finegrs_action_incremental(self,motorCommand,motorIndices):
        # todo: motorCommands is current joint states - last time joint states
        current_states_all = p.getJointStates(self.self_reconstructed_hand_Uid, motorIndices)
        for i in range(len(motorIndices)):
            p.setJointMotorControl2(self.self_reconstructed_hand_Uid,
                                    motorIndices[i],
                                    p.POSITION_CONTROL,
                                    targetPosition=current_states_all[i][0] + motorCommand,
                                    force=self.maxForce)

    def apply_fingers_action_absolute(self,motorCommand,motorIndices):
        for i in range(len(motorIndices)):
            p.setJointMotorControl2(self.self_reconstructed_hand_Uid,
                                    motorIndices[i],
                                    p.POSITION_CONTROL,
                                    targetPosition=motorCommand[i],
                                    force=self.maxForce)

    def get_force_convex_hull_minimum_distance(self,friction_coefficient):
        start_time=time.time()
        contact_position=[]
        contact_normal=[]
        #todo:because we should collect the contact points from the plam, the index should begin with -1
        for i in range(self.numJoints+1):
            #todo:Note, this contact normal direction is towards to object!!!
            contact=p.getContactPoints(self.object_Uid,self.self_reconstructed_hand_Uid, -1, i-1)
            # print ('link {0} has {1} points'.format(i-1,len(contact)))
            for j in range(len(contact)):
                contact_position.append(contact[j][5])
                contact_normal.append(contact[j][7])
                p.addUserDebugLine(contact[j][5],[contact[j][5][0]-contact[j][7][0],contact[j][5][1]-contact[j][7][1],
                                                  contact[j][5][2]-contact[j][7][2]],[0.0,0.0,1.0],lineWidth=1.0,lifeTime=100)

        object_position=p.getBasePositionAndOrientation(self.object_Uid)[0]
        print ('draw line time is {0}'.format(time.time()-start_time))

        if len(contact_position)==0:
            print ('The robotic hand did not grasp anything')
            force_convex_hull_distance_reward=-1
        else:
            print ('total contact point number is {0}'.format(len(contact_position)))
            start_time=time.time()
            for i in range(len(contact_position)):
                contact_plus_object_position=np.array(contact_position[i])-np.array(object_position)
                cross_matrix_about_contact_plus_object_position=np.array([[0,-contact_plus_object_position[2],contact_plus_object_position[1]],
                                                                          [contact_plus_object_position[2],0,-contact_plus_object_position[0]],
                                                                          [-contact_plus_object_position[1],contact_plus_object_position[0],0]])
                friction_cone_four_forces=get_friction_cone_vector(contact_normal[i],friction_coefficient)
                four_torques=np.dot(cross_matrix_about_contact_plus_object_position,friction_cone_four_forces)
                four_wrenches=np.vstack((friction_cone_four_forces,four_torques))
                if i==0:
                    force_convex_hull_points=four_wrenches.T
                else:
                    force_convex_hull_points=np.vstack((force_convex_hull_points,four_wrenches.T))
            print ('get points time is {0}'.format(time.time()-start_time))

            force_convex_hull_distance_reward=compute_force_convex_hull_function(force_convex_hull_points)
        return force_convex_hull_distance_reward

    def get_robotic_hand_minimum_energy(self,handstate,friction_coefficient,W_g,joint_torque_limit):
        contact_position_all=[]
        contact_normal_vector_all=[]
        joint_position_all=[]
        joint_orientation_all=[]
        current_index=0
        for i in range(len(handstate)):
            link_number=int(handstate[i].index(0)/2)
            contact_position=[]
            contact_normal_vector=[]
            joint_position=[]
            joint_orientation=[]
            for j in range(link_number):
                contact_position_link=[]
                contact_normal_vector_link=[]
                contact = p.getContactPoints(self.self_reconstructed_hand_Uid, self.object_Uid, current_index+j ,-1)
                for k in range(len(contact)):
                    contact_position_link.append(contact[k][5])
                    contact_normal_vector_link.append(contact[k][7])
                contact_position.append(contact_position_link)
                contact_normal_vector.append(contact_normal_vector_link)
                joint_position.append(p.getLinkState(self.self_reconstructed_hand_Uid, current_index+j)[0])
                joint_aixs_orientation=np.dot(np.array(p.getMatrixFromQuaternion(p.getLinkState(self.self_reconstructed_hand_Uid, current_index+j)[1])).reshape(3,3),
                                              np.array(p.getJointInfo(self.self_reconstructed_hand_Uid,current_index+j)[13]).reshape(-1,1))

                joint_orientation.append((joint_aixs_orientation[0][0],joint_aixs_orientation[1][0],joint_aixs_orientation[2][0]))
                #todo: draw joint axis
                p.addUserDebugLine(joint_position[-1],
                                   [joint_position[-1][0] - joint_orientation[-1][0], joint_position[-1][1] - joint_orientation[-1][1],
                                    joint_position[-1][2] - joint_orientation[-1][2]], [0.0, 1.0, 0.0], lineWidth=1.0, lifeTime=100)


            current_index=current_index+link_number
            contact_position_all.append(contact_position)
            contact_normal_vector_all.append(contact_normal_vector)
            joint_position_all.append(joint_position)
            joint_orientation_all.append(joint_orientation)

        object_position = p.getBasePositionAndOrientation(self.object_Uid)[0]
        N, J, G, link_number_sum, contact_number_sum=\
            integrate_all_finger_jocobian_cone_grasp_matrix(contact_position_all,contact_normal_vector_all,joint_position_all,
                                                            joint_orientation_all,object_position,friction_coefficient)
        W_g=[0,0,-W_g,0,0,0]
        joint_torque_limit=[joint_torque_limit]*link_number_sum
        if contact_number_sum!=0:
            energy_reward=get_minmum_energy_result(N, J, G, contact_number_sum, W_g, joint_torque_limit)
        else:
            energy_reward=-1
        print ('energy_reward is {0}'.format(energy_reward))
        return energy_reward


    def get_all_reward_for_one_handconfiguration(self,object_index_number):
        for i in range(object_index_number):
            if i==0:
                p.removeBody(self.object_Uid)
                self.object_index=i
                self.object_Uid, central_width, central_height = self.choose_object(self.object_index)
                for theta in [0,math.pi/12,math.pi/6,math.pi/4,math.pi/3,math.pi*5/12]:
                    for offset in [0.0,0.01,0.02,0.03,0.04]:
                        self.reset_hand_position_and_orientation(self.object_index, central_width, central_height, theta, offset)
                        self.finger_abduction_adduction_initial_degree = \
                            self.choose_inital_hand_abduction_adduction_joint_degree(self.object_index,
                                                                                     self.finger_abduction_adduction_index,
                                                                                     self.finger_abduction_adduction_in_which_side)
                        force_convex_hull_distance_reward, energy_reward=self.power_grasp_planner(0.2, 1, 20)



        

















if __name__ == '__main__':
    print ('1')
    p.connect(p.GUI)


    h=Self_reconstructed_hand()
    hand_Uid=h.self_reconstructed_hand_Uid

    numJoints = p.getNumJoints(hand_Uid)
    jointPositions = [0] * numJoints
    print(numJoints)


    p.setGravity(0, 0, -10)
    p.resetDebugVisualizerCamera(0.5, 150, -30, [0, 0, 0])

    motorsIds = []

    dv = 1.57
    for i in range(numJoints):
      a = 'joint_' + str(i + 1)
      motorsIds.append(p.addUserDebugParameter('joint_' + str(i + 1), -dv, dv, 0))


    def applyAction(motorCommands):
      for action in range(len(motorCommands)):
          motor = action
          p.setJointMotorControl2(hand_Uid,
                                  motor,
                                  p.POSITION_CONTROL,
                                  targetPosition=motorCommands[action],
                                  force=200)

    done = False
    while (not done):
      action = []
      for motorId in motorsIds:
          action.append(p.readUserDebugParameter(motorId))
      applyAction(action)

      # for i in [0,1]:
      #   print ('link {0} position is {1}, orientation is {2}, '
      #          'local position offset is {3}'.format(i, p.getLinkState(hand_Uid,i)[0],p.getLinkState(hand_Uid,i)[1],p.getLinkState(hand_Uid,i)[2]))
      #   print (p.getJointInfo(hand_Uid,i)[13],p.getJointInfo(hand_Uid,i)[14],p.getJointInfo(hand_Uid,i)[15])



        # print('world link {0} position is {1}, '
        #       'world orientation is {2}'.format(i, p.getLinkState(hand_Uid, i)[4], p.getLinkState(hand_Uid, i)[5]))
      p.stepSimulation()
  # while 1:
  #     continue