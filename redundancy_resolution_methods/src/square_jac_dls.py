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

class SquareJacDLS(JacobianPinv, object):

    def __init__(self):
        super(SquareJacDLS, self).__init__(NODE_NAME='SquareJacDLS')


    def __jacob_inv_dls(self, jacob):
        '''
        For calculating the inverse jacobian using damp least square
        '''
        j_transpose = jacob.transpose()
        dls = np.matmul(j_transpose, jacob) + (0.1*0.1*np.eye(self._nr_jnts))
        j_inv_dls = np.matmul(np.linalg.inv(dls), j_transpose)
        return j_inv_dls

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

            # Create j_bar
            zeros = np.zeros((2,6))
            eye = np.eye(2)
            bot = np.concatenate([eye, zeros], axis=1)
            jacob_e = np.copy(self._jacob_e)
            jac_bar = np.concatenate([jacob_e, bot], axis=0)


            v_bar = np.vstack((self._vel_inp, np.zeros((2,1))))
            jac_pinv = np.linalg.pinv(jac_bar)
            jac_inv_dls = self.__jacob_inv_dls(jac_bar)
            # v_bar[0] = -0.1
            dq = np.matmul(jac_inv_dls, v_bar)
            q = np.round(self._q + dq*dt, 5) # Position time step

            np.set_printoptions(precision=3, suppress=True)
            # print(dq[0:2])
            # print(" ")
            # print(jac_bar)
            # print(" ")

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
    obj = SquareJacDLS()
    obj.main()
