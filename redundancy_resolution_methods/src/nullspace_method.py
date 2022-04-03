#!/usr/bin/env python
import rospy
import tf
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from jacobian_pinv import JacobianPinv

class NullspaceMethod(JacobianPinv, object):

    def __init__(self):
        super(NullspaceMethod, self).__init__(NODE_NAME='nullspace_methods')

    def __manipulability_gradient__(self):
        # Create a matrix for individually moving each joint 
        h = 10^-4
        delta_matrix = np.zeros((self._nr_jnts,self._nr_jnts))
        np.fill_diagonal(delta_matrix, h)

        for k in range(self._nr_jnts):
            # Find the 'next; and 'previous' Jacobian given a movement in
            # the independent (joint) variable and numerically calculate
            # the jacobian wrt the independent variable
            jac_p = R.jacob0(curr_q + delta_matrix(k,:));
            jac_p = [jac_p(1:2,:); jac_p(6,:)];
            jac_m = R.jacob0(curr_q - delta_matrix(k,:));
            jac_m = [jac_m(1:2,:); jac_m(6,:)];
            partialJ_partialqk = (jac_p - jac_m)/(2*h);     

    def main(self):
        j0_pub = rospy.Publisher('/robot/joint0_position_controller/command', Float64, queue_size=1)
        j1_pub = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=1)
        j2_pub = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=1)
        j3_pub = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=1)
        j4_pub = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=1)
        j5_pub = rospy.Publisher('/robot/joint5_position_controller/command', Float64, queue_size=1)
        base_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=1)

        dt = 0.04 # Polyscope runs at 25Hz
        rate_limiter = rospy.Rate(25)

        while not rospy.is_shutdown():
            rate_limiter.sleep()

            jac_pinv = np.linalg.pinv(self._jacob_e)

            # WEIGHTS = np.power(np.array([50, 50, 1, 1, 1, 1, 1, 1]), -0.5)
            # WEIGHT_MATRIX = np.diag(WEIGHTS)
            # j_aux = np.matmul(self._jacob_e, WEIGHT_MATRIX)
            # jac_pinv = np.matmul(WEIGHT_MATRIX, np.linalg.pinv(j_aux))

            primary_vel = np.matmul(jac_pinv, self._vel_inp)
            q_vogui = np.zeros((self._nr_jnts,1))
            for i in range(2):
                q_vogui[i] = -primary_vel[i]

            nullspace_vel = np.matmul(np.eye(self._nr_jnts) - np.matmul(jac_pinv,self._jacob_e), q_vogui)

            dq = primary_vel + nullspace_vel
            q = np.round(self._q + dq*dt, 5) # Position time step

            base_vel = Twist()
            base_vel.linear.y = dq[0]
            base_vel.linear.x = dq[1]

            base_pub.publish(base_vel)
            j0_pub.publish(q[2])
            j1_pub.publish(q[3])
            j2_pub.publish(q[4])
            j3_pub.publish(q[5])
            j4_pub.publish(q[6])
            j5_pub.publish(q[7])

if __name__ == '__main__':
    obj = NullspaceMethod()
    obj.main()