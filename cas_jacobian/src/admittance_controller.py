#!/usr/bin/env python
import rospy
import numpy as np
import tf
import tf_conversions
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray, Float64

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.

    Input
    Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 

    Return: A 3x3 element matrix representing the full 3D rotation matrix. 

    This rotation matrix converts a point in the local reference 
    frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
           [r10, r11, r12],
           [r20, r21, r22],
           ])

    return rot_matrix

class AdmittanceController():
    '''
    An admittance controller for a robotic manipulator
    '''

    def __init__(self):
        rospy.init_node('admittance_controller')

        # Attributes used in subscriber callbacks
        self.__jacob0 =  np.zeros((6,6)) # Jacobian matrix in world frame
        self.__jacobe =  np.zeros((6,6)) # Jacobian matrix in ee frame relative to world
        self.__ft = np.zeros((6,1)) # Force exerted on the ee (from joystick)
        self.__q = np.zeros((6,1)) # Current joint states

        # Subscribers
        rospy.Subscriber("joy", Joy, self.joy_callback) # For obtaining joystick values 
        rospy.Subscriber("jacobian_0", Float64MultiArray, self.jacob0_callback) # For obtaining the world jacobian
        rospy.Subscriber("jacobian_e", Float64MultiArray, self.jacobe_callback) # For obtaining the ee jacbian
        rospy.Subscriber("robot/joint_states", JointState, self.robot_state_callback) # For obtaining the current robot state

    def robot_state_callback(self, msg):
        '''
        Save robot arm's current joint state. For whatever reason, the first three joints are
        in reverse order
        '''
        self.__q[0] = msg.position[2]
        self.__q[1] = msg.position[1]
        self.__q[2] = msg.position[0]
        self.__q[3] = msg.position[3]
        self.__q[4] = msg.position[4]
        self.__q[5] = msg.position[5]

    def joy_callback(self, msg):
        '''
        Obtain the current values on the joysticke
        '''
        self.__ft[0] = msg.axes[0]
        self.__ft[1] = msg.axes[1]
        self.__ft[2] = msg.axes[3]
        self.__ft[4] = msg.axes[4]

    def jacob0_callback(self, msg):
        '''
        Obtain the current jacobian of the robot arm
        '''
        idx = 0
        for i in range(6):
            for j in range(6):
                self.__jacob0[i,j] = msg.data[idx]
                idx = idx + 1

    def jacobe_callback(self, msg):
        '''
        Obtain the current jacobian of the robot arm
        '''
        idx = 0
        for i in range(6):
            for j in range(6):
                self.__jacobe[i,j] = msg.data[idx]
                idx = idx + 1

    def __jacob_inv_dls(self, jacob):
        '''
        For calculating the inverse jacobian using damp least square 
        '''
        j_transpose = jacob.transpose()
        dls = np.matmul(j_transpose, jacob) + (0.1*0.1*np.eye(6))
        j_dls = np.matmul(np.linalg.inv(dls), j_transpose)
        return j_dls


    def main(self):
        '''
        Main thread that continually moves the robot arm based on the joystick input
        '''
        # Publishers for position controller
        j0_pub = rospy.Publisher('/robot/joint0_position_controller/command', Float64, queue_size=1)
        j1_pub = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=1)
        j2_pub = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=1)
        j3_pub = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=1)
        j4_pub = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=1)
        j5_pub = rospy.Publisher('/robot/joint5_position_controller/command', Float64, queue_size=1)

        # Chaange the starting position of the robot so it isn't in singularity
        q_desired = np.array([[-0.87266, -1.2217, 1.2217, 0, 0, 0]]).transpose() # The desired joint setpoint
        while not rospy.is_shutdown():
            j0_pub.publish(-0.87266) # -50deg
            j1_pub.publish(-1.2217) # -70 deg
            j2_pub.publish(1.2217) # 70 deg
            j3_pub.publish(0) # -50deg
            j4_pub.publish(0) # -70 deg
            j5_pub.publish(0) # 70 deg
            if ((abs(self.__q - q_desired)) < 0.5).all():
                break

        ka = np.zeros((6,6)) # Virtual spring stiffness of the admittance controller
        kp = 0.2 # Error constant
        np.fill_diagonal(ka, 0.5) # Populate the virtual spring matrix

        dt = 0.1 # Timestep

        # [-0.06340057188217522, 0.00012566847434230884, 0.5545] # Robot base location
        rate_limiter = rospy.Rate(25)

        while not rospy.is_shutdown():
            rate_limiter.sleep()
            try:
                
                v_in = np.matmul(ka, self.__ft) # Input from human
                q_error = q_desired - self.__q # Error from desired setpoint
                jacobe_dls = self.__jacob_inv_dls(self.__jacobe) # Jacobian inverse with DLS in EE tool frame
                dq = np.matmul(jacobe_dls, v_in) + np.round(q_error*kp, 5) # The next required joint velocity
                q = np.round(self.__q + dq*dt, 5) # Position time step

                # Publish the joint states
                j0_pub.publish(q[0])
                j1_pub.publish(q[1])
                j2_pub.publish(q[2])
                j3_pub.publish(q[3])
                j4_pub.publish(q[4])
                j5_pub.publish(q[5])
            
            except Exception as e:
                print("not working")
                print(e)
                print("\n")


if __name__ == '__main__':
    cont = AdmittanceController()
    cont.main()
