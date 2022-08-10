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

        # Initialise robot model
        Robot = UrVoguiKdl()

        # Initialise publishers and subscribers required to control the arm and platform 
        ur_vel_pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)
        base_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=1)

        # Frequency of control loop
        frequency = 500.0 # e-series controlled at 500Hz
        dt = 1.0/frequency
        rate_limiter = rospy.Rate(frequency)
        

        # Parameters for calculating secondary task calculation
        tool_in_use = True
        tau = 0.27
        min_allowable_weight = 10.0**-5.0
        secondary_vel_gain = 3

        rospy.sleep(1.0)
        rospy.loginfo("Entering control loop of nullspace controller...")

        while not rospy.is_shutdown():
            # Initialise joint velocities of system
            ur_vel = Float64MultiArray()
            base_vel = Twist()
            base_vel.linear.y = 0
            base_vel.linear.x = 0
            ur_vel.data = 6*[0]

            # Save current values for present state
            curr_q = self._q.copy()
            curr_vel_in = self._vel_inp.copy()

            # Calculate manipulability of manipulator
            manipulability = Robot.arm_maniplty(curr_q)

            # Obtain gamma, used to calculate the weighting for secondary task
            if not tool_in_use:
                # Only the platform is utilised
                platform_weight = min_allowable_weight
                arm_weight = 1.0
            else:
                # The arm is utilised depending on the manipulability
                beta = 0.02
                beta_power = (manipulability + min_allowable_weight) / tau
                arm_weight = pow(beta, beta_power)
                platform_weight = 1.0 - arm_weight

            # Calculate Jacobian in tool frame
            jacob_e = Robot.Jacob(curr_q, True)

            # Weighted Jacobian based on manipulability value
            jac_pinv = self.weighted_jacobian([platform_weight if i < 2 else arm_weight for i in range(self._nr_jnts)], jacob_e)

            # Weighted Jacobian that utilises manipulator. Used for secondary task in nullspace projection
            jac_pinv_arm = self.weighted_jacobian([1.0 if i < 2 else min_allowable_weight for i in range(self._nr_jnts)], jacob_e)

            # Enclose velocity calc in if statement so any small input errors does not accumulate
            if np.max(np.abs(curr_vel_in)) > 0:
                # Primary velocity to satisfy the primary task
                primary_vel = np.matmul(jac_pinv, curr_vel_in)

                # Secondary task definition
                improve_gradient = Robot.manipulability_gradient(curr_q, False)

                # Secondary velocity to satisfy the secondary task, performed in the nullspace
                secondary_vel = np.matmul(np.eye(self._nr_jnts) - np.matmul(jac_pinv_arm,jacob_e), improve_gradient)

                # Calculate required joint velocity
                dq = primary_vel + secondary_vel_gain*secondary_vel

                # Cap the maximum velocity of the system
                for idx, val in enumerate(dq):
                    if idx > 1: # Cap UR10s velocity
                        dq[idx] = copysign(0.25, val) if val > 1.0 else val
                    else: # Cap Vogui's velocity
                        dq[idx] = copysign(0.5, val) if val > 1.0 else val
                dq[:2] = np.where(abs(dq[:2]) < 0.02, 0, dq[:2]) # Zero negligible velocity of the platform, if any 

                # Publish joint velocities
                base_vel.linear.y = dq[0]
                base_vel.linear.x = dq[1]
                ur_vel.data = dq[2:]

            base_pub.publish(base_vel)
            ur_vel_pub.publish(ur_vel)

            rate_limiter.sleep()


if __name__ == '__main__':
    obj = NullspaceMethod()
    obj.main()

