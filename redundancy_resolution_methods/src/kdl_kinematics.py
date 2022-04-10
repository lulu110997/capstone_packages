#!/usr/bin/env python
from __future__ import division
from math import pi as M_PI

import numpy as np
import rospy
import PyKDL as kdl
import kdl_parser_py.urdf

# The following 3 functinons were extracted from https://github.com/RethinkRobotics/baxter_pykdl/blob/master/src/baxter_kdl/kdl_kinematics.py
def kdl_to_mat(m):
    mat =  np.mat(np.zeros((m.rows(), m.columns())))
    for i in range(m.rows()):
        for j in range(m.columns()):
            mat[i,j] = m[i,j]
    return mat

def joint_kdl_to_list(q):
    if q == None:
        return None
    return [q[i] for i in range(q.rows())]

def joint_list_to_kdl(q):
    if q is None:
        return None
    if type(q) == np.matrix and q.shape[1] == 0:
        q = q.T.tolist()[0]
    q_kdl = kdl.JntArray(len(q))
    for i, q_i in enumerate(q):
        q_kdl[i] = q_i
    return q_kdl

class KDLKinematics(object):
    '''
    KDL does not currently support mobile manipulators. To calculate the
    kinematics of the robot, we need to model the mobile base as two 
    prismatic joints. This class stores a KDL chain which models the mobile
    base of the Vogui as two prismatic joints.

    This class also stores useful methods which
    - Calculates forward kinematics
    - Calculates inverse kinematics
    - Calculates Jacobian in world and EE frame
    - Calculates the partial derivative of the Jacobian wrt each joint (used 
      for calculation of manipulability gradient)

    Please ensure that the joint positions are in the form of [y_base_pose, 
    x_base_pos, q1...q6] and is a column (numpy) array
    '''

    def __init__(self, urdf_file=None):
        '''
        Create a model of the RB Vogui with the UR10 on top as a KDL chain
        '''

        self.vogui_model = kdl.Chain() # Define the chain that the model will be stored in
        self.nr_jnts = 8 # of arm = 6 + 2 DoF from mobile base

        # Use KDL parser to obtain a KDL tree of the RB Vogui
        ok = False # Bool that checks if a tree has been created successfully or not
        try: # A urdf file exists
            ok, vogui_tree_ = kdl_parser_py.urdf.treeFromFile(urdf_file)
        except: # Get robot description from parameter server as there is no urdf file
            robot_desc_ = rospy.get_param("/robot/robot_description")
            ok, vogui_tree_ = kdl_parser_py.urdf.treeFromString(robot_desc_)
        finally:
            # Check a tree has been created
            if not ok:
                raise Exception("Cannot create tree from URDF file or robot description")
                return

        # Define the base link and end link (obtained from the URDF) that will define the chain from the tree
        base_link = "robot_arm_base"
        end_link = "robot_arm_tool0"

        # Obtain a KDL chain from the KDL tree. This is the UR10
        UR10_chain = vogui_tree_.getChain(base_link, end_link)

        # Create a KDL vector to define the xy_origin (0,0,0), prisy_axis (0,1,0) and  prix_axis (1,0,0)
        # kdl vectors
        xy_origin = kdl.Vector(0,0,0)
        prisy_axis = kdl.Vector(0,1,0)
        prisx_axis = kdl.Vector(1,0,0)

        # Model the RB_VOGUI using 2 prismatic joint along the y and x axis
        #kdl joints
        prisy = kdl.Joint("prisy", xy_origin, prisy_axis, kdl.Joint.TransAxis)
        prisx = kdl.Joint("prisx", xy_origin, prisx_axis, kdl.Joint.TransAxis)


        # Define the segment in which each joint will be attached to then add those segments onto the chain
        # The x-segment that the arm will attach to needs to have the same frame as the base arm segment.
        seg_rot = kdl.Rotation()
        seg_rot.DoRotZ(M_PI)
        seg_vec = kdl.Vector()
        seg_frame = kdl.Frame(seg_rot, seg_vec)
        #kdl segments
        segy = kdl.Segment("segy", prisy)
        segx = kdl.Segment("segx", prisx, seg_frame)


        # Add the mobile base to the vogui model as two segments with prismatic joints, then add the UR10 to the chain
        self.vogui_model.addSegment(segy)
        self.vogui_model.addSegment(segx)
        self.vogui_model.addChain(UR10_chain)

        # Define kdl kinematic solvers
        self.fk_solver_ = kdl.ChainFkSolverPos_recursive(self.vogui_model) 
        self.jac_solver_ = kdl.ChainJntToJacSolver(self.vogui_model)

        # for i in range(0, self.vogui_model.getNrOfSegments()):
        #         print(self.vogui_model.getSegment(i).getJoint().getName())

    def Jacob(self, q, ee_frame=False):
        '''
        Calculates the Jacobian in either the world frame or EE frame. Default is world
        Returns the Jacobian matrix in the desired frame of ref as a np matrix

        ee_frame: bool. True if Jac in EE is desired
        f: desired frame of reference ('world' or 'ee')
        '''

        j_kdl = kdl.Jacobian(self.nr_jnts)
        q_kdl = joint_list_to_kdl(q)
        self.jac_solver_.JntToJac(q_kdl, j_kdl)

        if ee_frame:

            base_to_ee_rot = kdl.Rotation()
            seg_idx = 0
            jnt_idx = 0

            while seg_idx < self.vogui_model.getNrOfSegments():

                if self.vogui_model.getSegment(seg_idx).getJoint().getType() == 8:
                    base_to_ee_rot = base_to_ee_rot * self.vogui_model.getSegment(seg_idx).pose(0).M
                    seg_idx = seg_idx + 1

                else:
                    base_to_ee_rot = base_to_ee_rot * self.vogui_model.getSegment(seg_idx).pose(q_kdl[jnt_idx]).M
                    seg_idx = seg_idx + 1
                    jnt_idx = jnt_idx + 1

            base_to_ee_rot.SetInverse() # Need the rotation from EE to base
            j_kdl.changeBase(base_to_ee_rot) # Change the frame of reference of the Jacobian
        # print(j_kdl)
        return kdl_to_mat(j_kdl)

    def fkine(self, q):
        '''
        Forward kinematics on the given joint angles, returning a homogenous transform of the
        EE's pose in cartesian space wrt the origin (world frame of reference)
        '''
        ee_frame = kdl.Frame()
        ret = self.fk_solver_.JntToCart(joint_list_to_kdl(q), ee_frame)

        if ret >= 0:
            p = ee_frame.p
            M = ee_frame.M
            return np.mat([[M[0,0], M[0,1], M[0,2], p.x()], 
                           [M[1,0], M[1,1], M[1,2], p.y()], 
                           [M[2,0], M[2,1], M[2,2], p.z()],
                           [     0,      0,      0,     1]])
        else:
            return None

    def manipulability_gradient(self, q, ee_frame=False):
        '''
        Uses a numerical approach to find the Jacobian Derivative wrt to the position of each joint. This
        is then used to compute the manipulability at this specific joint configuration
        '''
        # import time
        # start = time.time()

        # Create a matrix where each column represents a small (h) movement of an independent variable
        numerical_dm_dq_ = np.zeros((self.nr_jnts,1)) # Manipulability gradient
        delta_matrix = np.zeros((self.nr_jnts,self.nr_jnts))
        h = 0.0001
        np.fill_diagonal(delta_matrix, h)

        for k in range(self.nr_jnts):

            # Obtain the current manipulability, Jacobian and its 
            # pseudoinverse for the current joint config
            curr_jac = self.Jacob(q, ee_frame)
            curr_jac_pinv = np.linalg.pinv(curr_jac)
            curr_mani = np.sqrt(np.linalg.det(curr_jac*np.transpose(curr_jac)))

            # Find the 'next' and 'previous' Jacobian given a movement in
            # the independent (joint) variable and numerically calculate
            # the jacobian wrt the independent variable
            q_h = np.transpose(np.array([delta_matrix[:,k]])) # Need to turn the array into a column vector
            jac_p = self.Jacob(q + q_h, ee_frame) 
            jac_m = self.Jacob(q - q_h, ee_frame) 

            partialJ_partialqk = (jac_p - jac_m)/(2*h)
            numerical_dm_dq_[k] = curr_mani*np.trace(partialJ_partialqk*curr_jac_pinv)

        # print(curr_mani)
        # print(1 / (time.time()-start))
        return numerical_dm_dq_

if __name__ == '__main__': 
    # q0 = np.array([[0, 0, -0.87266, -1.2217, 1.2217, 0, 0, 0]])
    q0 = np.array([0, 0, 0, -0.5230, 1.2215, 2.9665, 1.5705, 0]) # A joint config with high manipulability (2.16)
    ob = KDLKinematics(urdf_file='/home/louis/catkin_ws/src/custom_ur_control/rb_vogui_generated.urdf')
    ob.fkine(q0)
    ob.Jacob(q0)
    ob.Jacob(q0, True)
    ob.manipulability_gradient(q0)

'''

q0 = np.array([[0, 0, -0.87266, -1.2217, 1.2217, 0, 0, 0]])

j0 =

    0.0000    1.0000    0.4343    0.2957   -0.0744   -0.0744    0.0593         0
    1.0000   -0.0000    0.6988   -0.3524    0.0886    0.0886   -0.0706         0
    0.0000   -0.0000    0.0000   -0.7819   -0.5723    0.0000   -0.0000         0
         0         0    0.0000    0.7660    0.7660    0.7660   -0.0000    0.7660
         0         0   -0.0000    0.6428    0.6428    0.6428    0.0000    0.6428
         0         0    1.0000    0.0000    0.0000    0.0000   -1.0000    0.0000


je =

    0.7660   -0.6428    0.2561   -0.4600    0.1157    0.1157   -0.0922         0
   -0.0000    0.0000   -0.0000   -0.7819   -0.5723         0         0         0
    0.6428    0.7660    0.7819         0         0         0         0         0
         0         0         0         0         0         0         0         0
         0         0    1.0000         0         0         0   -1.0000         0
         0         0    0.0000    1.0000    1.0000    1.0000    0.0000    1.0000

'''