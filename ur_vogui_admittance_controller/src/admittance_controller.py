#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class AdmittanceController(object):
    '''
    An admittance controller for a robotic manipulator
    '''

    def __init__(self):
        rospy.init_node('admittance_controller')

        # Attributes
        self.__task_space = 6
        self.__ft = np.zeros((self.__task_space,1)) # Force exerted on the ee (from joystick)

        # Subscriber
        rospy.Subscriber("joy", Joy, self.joy_callback) # For obtaining joystick values

    def joy_callback(self, msg):
        '''
        Obtain the current values on the joysticke
        '''
        self.__ft[0] = msg.axes[0]
        self.__ft[1] = msg.axes[1]
        self.__ft[2] = msg.axes[3]
        self.__ft[4] = msg.axes[4]

    def main(self):
        '''
        Main thread that continually moves the robot arm based on the joystick input
        '''

        ka = np.zeros((self.__task_space,self.__task_space)) # Virtual spring stiffness of the admittance controller
        stiffness = 0.5

        np.fill_diagonal(ka, stiffness) # Populate the virtual spring matrix

        rate_limiter = rospy.Rate(25) # The same rate that the HARDWARE is running at

        v_in_pub = rospy.Publisher("velocity_input", Float64MultiArray, queue_size=1)

        while not rospy.is_shutdown():
            
            rate_limiter.sleep()
            
            v_in = np.matmul(ka, self.__ft) # Input from human

            v_in_msg = Float64MultiArray()
            for i in range(self.__task_space):
                v_in_msg.data.append(v_in[i])

            v_in_pub.publish(v_in_msg)


if __name__ == '__main__':
    cont = AdmittanceController()
    cont.main()
