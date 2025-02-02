#!/usr/bin/env python
from __future__ import division
import rospy
import tf
import numpy as np
from math import copysign
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from ur_vogui_kdl import UrVoguiKdl
from jacobian_pinv import JacobianPinv

class NullspaceMethod(JacobianPinv, object):

    def __init__(self):
        super(NullspaceMethod, self).__init__(NODE_NAME='nullspace_methods')


    def reduced_gradient_method(self):
        return

    @staticmethod
    def weighted_jacobian(weight, jacobian):
        '''
        weight: list of weights to fill the diagonal of a matrix. Must be > 0 so
                that a positive definite and symmetric matrix can be obtained. 
                The larger the weight is, the more the corresponding joint gets 
                penalised.
        jacobian: jacobian to be weighted

        Returns a the weighted pseudoinverse matrix that minimises the norm of the
        joint velocity vector based on the weighting matrix
        '''
        # Obtain the weighting matrix
        weights_sqrt = np.power(np.array(weight), -0.5)
        weights_matrix_sqrt = np.diag(weights_sqrt)

        # Obtain the weighted pseudoinverse using an auxiliary matrix
        j_aux = np.matmul(jacobian, weights_matrix_sqrt)
        jac_pinv = np.matmul(weights_matrix_sqrt, np.linalg.pinv(j_aux))

        return jac_pinv

    def main(self):

        Robot = UrVoguiKdl()

        # j0_pub = rospy.Publisher('/robot/joint0_position_controller/command', Float64, queue_size=1)
        # j1_pub = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=1)
        # j2_pub = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=1)
        # j3_pub = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=1)
        # j4_pub = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=1)
        # j5_pub = rospy.Publisher('/robot/joint5_position_controller/command', Float64, queue_size=1)
        ur_vel_pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)
        base_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=1)
        ur_vel = Float64MultiArray()
        base_vel = Twist()

        # Physical robot
        # import actionlib
        # from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
        # from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
        # import sys

        # trajectory_client = actionlib.SimpleActionClient("/scaled_pos_joint_traj_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        # if not trajectory_client.wait_for_server(rospy.Duration(5.0)):
        #     print("Could not reach controller action server")
        #     sys.exit(-1)

        # JOINT_NAMES = ["robot_arm_shoulder_pan_joint", "robot_arm_shoulder_lift_joint", "robot_arm_elbow_joint", "robot_arm_wrist_1_joint",
        #                "robot_arm_wrist_2_joint", "robot_arm_wrist_3_joint", ]
        # Physical robot

        # May need to have a close loop controller for sim. In real life, human is the controller
        frequency = 500.0 # e-series controlled at 500Hz
        dt = 1.0/frequency
        rate_limiter = rospy.Rate(frequency)
        
        # To do: subscribe to /joy and use button to simulate tool in use
        tool_in_use = True
        tau = 0.2
        min_allowable_weight = 10.0**-5.0

        # Gain for the secondary velocity
        secondary_vel_gain = 2

        rospy.sleep(1.0)

        print("READY")

        while not rospy.is_shutdown():
            rate_limiter.sleep()

            # Save current values for present state
            curr_q = self._q.copy()
            curr_vel_in = self._vel_inp.copy()

            # Calculate manipulability of manipulator
            manipulability = Robot.arm_maniplty(curr_q)

            # Obtain gamma
            if not tool_in_use:
                platform_weight = min_allowable_weight
                arm_weight = 1.0
            else:
                beta = 0.02
                beta_power = (manipulability + min_allowable_weight) / tau
                arm_weight = pow(beta, beta_power)
                platform_weight = 1.0 - arm_weight

            # Calculate Jacobian in tool frame
            jacob_e = Robot.Jacob(curr_q, True)

            # Weighted Jacobian based on manipulability value
            jac_pinv = self.weighted_jacobian([platform_weight if i < 2 else arm_weight for i in range(self._nr_jnts)], jacob_e)

            # Weighted Jacobian that utilises manipulator. Used for secondary task in nullspace projection
            jac_pinv_arm = self.weighted_jacobian([1-min_allowable_weight if i < 2 else min_allowable_weight for i in range(self._nr_jnts)], jacob_e)

            # Enclose velocity calc in if statement so the small input errors does not accumulate
            if np.max(np.abs(curr_vel_in)) > 0:
                # Primary velocity to satisfy the primary task
                primary_vel = np.matmul(jac_pinv, curr_vel_in)

                # Secondary task definition
                improve_gradient = Robot.manipulability_gradient(curr_q, False)

                # Secondary velocity to satisfy the secondary task, performed in the nullspace
                secondary_vel = np.matmul(np.eye(self._nr_jnts) - np.matmul(jac_pinv_arm,jacob_e), improve_gradient)

                # Calculate velocity
                dq = primary_vel + secondary_vel_gain*secondary_vel
                # dq[:2] = np.where(abs(dq[:2]) < 0.09, 0, dq[:2]) # Zero negligible velocity of the platform, if any 
                for idx, val in enumerate(dq[2:]):
                    dq[2+idx] = val if abs(val)<1.5 else copysign(0.5, val)
                base_vel.linear.y = dq[0]
                base_vel.linear.x = dq[1]
                ur_vel.data = dq[2:]

                # Position time step
                # q = np.round(curr_q + dq*dt, 5)

                # Publish velocities and position for this timestep
                base_pub.publish(base_vel)
                # print(ur_vel)
                ur_vel_pub.publish(ur_vel)
                # j0_pub.publish(q[2])
                # j1_pub.publish(q[3])
                # j2_pub.publish(q[4])
                # j3_pub.publish(q[5])
                # j4_pub.publish(q[6])
                # j5_pub.publish(q[7])

                # Physical robot
                # Create and fill trajectory goal
                # goal = FollowJointTrajectoryGoal()
                # goal.trajectory.joint_names = JOINT_NAMES
                # position_list = [[q[2], q[3], q[4], q[5], q[6], q[7]]]
                # duration_list = [dt]

                # point = JointTrajectoryPoint()
                # point.positions = position_list[0]
                # point.time_from_start = rospy.Duration(duration_list[0])
                # goal.trajectory.points.append(point)

                # trajectory_client.send_goal(goal)
                # trajectory_client.wait_for_result()
                # Physical robot
            else:
                ur_vel = Float64MultiArray()
                ur_vel.data = [0,0,0,0,0,0]
                ur_vel_pub.publish(ur_vel)



if __name__ == '__main__':
    obj = NullspaceMethod()
    obj.main()

