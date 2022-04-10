#!/usr/bin/env python
import rospy
import tf
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from kdl_kinematics import KDLKinematics
from jacobian_pinv import JacobianPinv

class NullspaceMethod(JacobianPinv, object):

    def __init__(self):
        super(NullspaceMethod, self).__init__(NODE_NAME='nullspace_methods')

    def main(self):

        Robot = KDLKinematics()

        j0_pub = rospy.Publisher('/robot/joint0_position_controller/command', Float64, queue_size=1)
        j1_pub = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=1)
        j2_pub = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=1)
        j3_pub = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=1)
        j4_pub = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=1)
        j5_pub = rospy.Publisher('/robot/joint5_position_controller/command', Float64, queue_size=1)
        base_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=1)

        frequency = 25.0 # Polyscope runs at 25Hz
        dt = 1.0/frequency
        rate_limiter = rospy.Rate(frequency)

        while not rospy.is_shutdown():
            rate_limiter.sleep()

            curr_q = self._q.copy()
            curr_vel_in = self._vel_inp.copy()

            jacob_e = Robot.Jacob(curr_q, True)

            WEIGHTS = np.power(np.array([10, 10, 1, 1, 1, 1, 1, 1]), -0.5)
            WEIGHT_MATRIX = np.diag(WEIGHTS)
            j_aux = np.matmul(jacob_e, WEIGHT_MATRIX)
            jac_pinv = np.matmul(WEIGHT_MATRIX, np.linalg.pinv(j_aux))

            if np.max(np.abs(curr_vel_in)) > 0:
                primary_vel = np.matmul(jac_pinv, curr_vel_in)
                secondary_vel = np.matmul(np.eye(self._nr_jnts) - np.matmul(jac_pinv,jacob_e), Robot.manipulability_gradient(curr_q, True))

                dq = primary_vel + secondary_vel
                q = np.round(curr_q + dq*dt, 5) # Position time step

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