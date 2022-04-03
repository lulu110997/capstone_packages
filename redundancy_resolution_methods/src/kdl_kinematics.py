#!/usr/bin/env python
from urdf_parser_py.urdf import URDF
from math import pi as M_PI

import numpy as np
import rospy
import PyKDL as kdl
import kdl_parser_py.urdf
import tf

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
    - Calculates the partial derivative of the Jacobian wrt each joint (used for calculation of manipulability gradient)

    Please ensure that the joint positions are in the form of [y_base_pose, x_base_pos, q1...q6]
    '''

    def __init__(self):
        '''
        Create a model of the RB Vogui with the UR10 on top as a KDL chain
        '''

        self.vogui_model = kdl.Chain() # Define the chain that the model will be stored in
        self.nr_jnts = 8 # of arm = 6 + 2 DoF from mobile base

        # Get robot description from parameter server
        robot_desc_ = rospy.get_param("/robot/robot_description")

        # Use KDL parser to obtain a KDL tree of the RB Vogui from the robot desc
        ok, vogui_tree_ = kdl_parser_py.urdf.treeFromString(robot_desc_)
        if not ok:
            raise Exception("Cannot create tree from robot robot description")
            return

        # Define the base link and end link (obtained from the URDF) that will define the chain from the tree
        base_link = "robot_arm_base"
        end_link = "robot_arm_tool0"

        # Obtain a KDL chain from the KDL tree
        vogui_chain_ = vogui_tree_.getChain(base_link, end_link)

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
        #kdl segments
        segy = kdl.Segment("segy", prisy)
        segx = kdl.Segment("segx", prisx)

        # Add the mobile base to the vogui model as two segments with prismatic joints
        self.vogui_model.addSegment(segy)
        self.vogui_model.addSegment(segx)
        for i in range(1, 8): # Add each segment of the robot arm on the chain, remove segments that has joint type None
            self.vogui_model.addSegment(vogui_chain_.getSegment(i))

        # Define kdl kinematic solvers
        self.fk_solver_ = kdl.ChainFkSolverPos_recursive(self.vogui_model) 
        self.jac_solver_ = kdl.ChainJntToJacSolver(self.vogui_model)

    def Jacob(self, q, world_to_ee_rot=None):
        '''
        Calculates the Jacobian in either the world frame or EE frame. Default is world
        Returns the Jacobian matrix in the desired frame of ref as a np matrix

        world_to_ee_rot: current joint position either as a list or a np array
        f: desired frame of reference ('world' or 'ee')
        '''

        j_kdl = kdl.Jacobian(self.nr_jnts)
        q_kdl = joint_list_to_kdl(q)
        self.jac_solver_.JntToJac(q_kdl, j_kdl)

        if f != 'world':
            listener = tf.TransformListener()
            world_to_ee_rotation = kdl.Rotation()
            while True:
                try:
                    (trans, rot) = listener.lookupTransform("robot_arm_tool0", "robot_arm_base", rospy.Time(0))
                    world_to_ee_rotation = rot
                    world_to_ee_rotation.DoRotZ(M_PI)
                    j_kdl.changeBase(world_to_ee_rotation)
                    break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
        print(j_kdl)
        return kdl_to_mat(j_kdl)

    def PartialJPartialQ(self, q):
        '''
        Uses a numerical approach to find the Jacobian Derivative wrt to the position of each joint
        '''

q0 = np.array([[0, 0, -0.87266, -1.2217, 1.2217, 0, 0, 0]]).transpose()
ob = KDLKinematics()
ob.Jacob(q0, 'ee')