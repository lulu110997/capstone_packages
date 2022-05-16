#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class JacobianPinv(object):

    def __init__(self, NODE_NAME='jacobian_pinv'):
        rospy.init_node(NODE_NAME)

        # Attributes
        self._nr_jnts = 8
        self._task_space = 6
        self._q = np.zeros((self._nr_jnts,1))
        self._vel_inp = np.zeros((self._task_space,1))
        self._jacob_e = np.zeros((self._task_space, self._nr_jnts))

        # Subscriber
        rospy.Subscriber("velocity_input", Float64MultiArray, self._vel_inp_cb)
        rospy.Subscriber("jacobian_e", Float64MultiArray, self._jacobian_e_cb)
        rospy.Subscriber("robot/joint_states", JointState, self._joint_states_cb)
        rospy.Subscriber("robot/robotnik_base_control/odom", Odometry, self._odom_cb)
        j0_pub = rospy.Publisher('/robot/joint0_position_controller/command', Float64, queue_size=1)
        j1_pub = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=1)
        j2_pub = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=1)
        j3_pub = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=1)
        j4_pub = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=1)
        j5_pub = rospy.Publisher('/robot/joint5_position_controller/command', Float64, queue_size=1)

        starting_pos = np.array([[0, 0, -0.2513, -1.2566, 1.2566, -1.5708, 1.5708, -1.5708]]).transpose()

        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            j0_pub.publish(starting_pos[2])
            j1_pub.publish(starting_pos[3])
            j2_pub.publish(starting_pos[4])
            j3_pub.publish(starting_pos[5])
            j4_pub.publish(starting_pos[6])
            j5_pub.publish(starting_pos[7])
            if (abs(self._q[2:] - starting_pos[2:]) < 0.01).all():
                break

    def _vel_inp_cb(self, msg):
        for i in range(len(msg.data)):
            self._vel_inp[i] = msg.data[i]

    def _jacobian_e_cb(self, msg):
        idx = 0
        for i in range(self._task_space):
            for j in range(self._nr_jnts):
                self._jacob_e[i,j] = msg.data[idx]
                idx = idx + 1

    def _joint_states_cb(self, msg):
        self._q[2] = msg.position[2]
        self._q[3] = msg.position[1]
        self._q[4] = msg.position[0]
        self._q[5] = msg.position[3]
        self._q[6] = msg.position[4]
        self._q[7] = msg.position[5]

    def _odom_cb(self, msg):
        self._q[0] = msg.pose.pose.position.y
        self._q[1] = msg.pose.pose.position.x

    def main(self):
        j0_pub = rospy.Publisher('/robot/joint0_position_controller/command', Float64, queue_size=1)
        j1_pub = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=1)
        j2_pub = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=1)
        j3_pub = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=1)
        j4_pub = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=1)
        j5_pub = rospy.Publisher('/robot/joint5_position_controller/command', Float64, queue_size=1)
        base_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=1)
        base_vel = Twist()

        dt = 0.04 # Polyscope runs at 25Hz
        rate_limiter = rospy.Rate(25)

        while not rospy.is_shutdown():
            rate_limiter.sleep()

            # WEIGHTS = np.power(np.array([50, 50, 1, 1, 1, 1, 1, 1]), -0.5)
            # WEIGHT_MATRIX = np.diag(WEIGHTS)
            # j_aux = np.matmul(self._jacob_e, WEIGHT_MATRIX)
            # jac_pinv = np.matmul(WEIGHT_MATRIX, np.linalg.pinv(j_aux))

            jac_pinv = np.linalg.pinv(self._jacob_e)
            dq = np.matmul(jac_pinv, self._vel_inp)
            q = np.round(self._q + dq*dt, 5) # Position time step

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
    obj = JacobianPinv()
    obj.main()