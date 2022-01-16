#!/usr/bin/env python
import rospy
import tf
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class Jacobian_Pinv():

    def __init__(self):
        rospy.init_node('jacobian_pinv')

        # Attributes
        self._nr_jnts = 9
        self._task_space = 6
        self._q = np.zeros((self._nr_jnts,1))
        self._vel_inp = np.zeros((self._task_space,1))
        self._jacob_e = np.zeros((self._task_space, self._nr_jnts))

        # Subscriber
        rospy.Subscriber("velocity_input", Float64MultiArray, self._vel_inp_cb)
        rospy.Subscriber("jacobian_e", Float64MultiArray, self._jacobian_e_cb)
        rospy.Subscriber("robot/joint_states", JointState, self._joint_states_cb)
        rospy.Subscriber("robot/robotnik_base_control/odom", Odometry, self._odom_cb)

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
        self._q[3] = msg.position[2]
        self._q[4] = msg.position[1]
        self._q[5] = msg.position[0]
        self._q[6] = msg.position[3]
        self._q[7] = msg.position[4]
        self._q[8] = msg.position[5]

    def _odom_cb(self, msg):
        orientation_msg = msg.pose.pose.orientation
        orientation = [orientation_msg.x, orientation_msg.y, orientation_msg.z, orientation_msg.w]
        __, __, yaw = tf.transformations.euler_from_quaternion(orientation)
        self._q[0] = yaw
        self._q[1] = msg.pose.pose.position.y
        self._q[2] = msg.pose.pose.position.x

    def main(self):
        j0_pub = rospy.Publisher('/robot/joint0_position_controller/command', Float64, queue_size=1)
        j1_pub = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=1)
        j2_pub = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=1)
        j3_pub = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=1)
        j4_pub = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=1)
        j5_pub = rospy.Publisher('/robot/joint5_position_controller/command', Float64, queue_size=1)
        base_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=1)

        starting_pos = np.array([[0, 0, 0, -0.87266, -1.2217, 1.2217, 0, 0, 0]]).transpose()
        
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            j0_pub.publish(-0.87266)
            j1_pub.publish(-1.2217)
            j2_pub.publish(1.2217)
            j3_pub.publish(0)
            j4_pub.publish(0)
            j5_pub.publish(0)
            if (abs(self._q[3:] - starting_pos[3:]) < 0.01).all():
                break

        dt = 0.04 # Polyscope runs at 25Hz
        rate_limiter = rospy.Rate(25)

        while not rospy.is_shutdown():
            rate_limiter.sleep()

            WEIGHTS = np.power(np.array([50, 50, 50, 1, 1, 1, 10, 10, 10]), -0.5)
            WEIGHT_MATRIX = np.diag(WEIGHTS)
            j_aux = np.matmul(self._jacob_e, WEIGHT_MATRIX)
            jac_pinv = np.matmul(WEIGHT_MATRIX, np.linalg.pinv(j_aux))

            # jac_pinv = np.linalg.pinv(self._jacob_e)
            dq = np.matmul(jac_pinv, self._vel_inp)
            q = np.round(self._q + dq*dt, 5) # Position time step

            base_vel = Twist()
            base_vel.angular.z = dq[0]
            base_vel.linear.y = dq[1]
            base_vel.linear.x = dq[2]

            base_pub.publish(base_vel)
            j0_pub.publish(q[3])
            j1_pub.publish(q[4])
            j2_pub.publish(q[5])
            j3_pub.publish(q[6])
            j4_pub.publish(q[7])
            j5_pub.publish(q[8])

if __name__ == '__main__':
    obj = Jacobian_Pinv()
    obj.main()

# import numpy as np
# import math
# a = np.array([[4,0], [3,-5]])

# u,s,v = np.linalg.svd(a)
# s = np.diag(s)

# print(u)
# print(s)
# print(v)
# print((np.round(u@s@v,5)==a).all())
# WEIGHTS = np.power(np.array([100,100,100,1,1,1,1,1,1]), -0.5)
# WEIGHT_MATRIX = np.diag(WEIGHTS)
# print(WEIGHTS)
# print(100**-0.5)