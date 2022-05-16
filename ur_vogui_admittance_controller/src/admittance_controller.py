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
        self.__ft[0] = msg.axes[0] # Left right of left joystick mapped to x-axis transl
        self.__ft[1] = msg.axes[1] # Up down of left joystick mapped to y-axis transl
        self.__ft[2] = msg.axes[4] # Left right of right joystick mapped to z-axis rot
        self.__ft[5] = msg.axes[3] # Up down of right joystick mapped to z-axis transl

    def main(self):
        '''
        Main thread that continually moves the robot arm based on the joystick input
        '''

        # Publisher for desired velocity in the cartesian space
        v_in_pub = rospy.Publisher("velocity_input", Float64MultiArray, queue_size=1)

        # Populate the virtual damping of the admittance controller
        kd = np.zeros((self.__task_space,self.__task_space))
        damping_manip_transl = 0.8
        damping_manip_rot = 0.5

        for i in range(self.__task_space):
            if i < 3:
                kd[i,i] = 1.0 / damping_manip_transl
            else:
                kd[i,i] = 1.0 / damping_manip_rot

        print("The damping matrix for the admittance controller is:")
        print(str(kd) + "\n")

        rate_limiter = rospy.Rate(300) # The same rate that the HARDWARE is running at

        while not rospy.is_shutdown():
            rate_limiter.sleep()

            v_in = np.matmul(kd, self.__ft) # Convert force to velocity input

            # Convert np to ros_msg and publish the desired cartesian velocity over a topic
            v_in_msg = Float64MultiArray()
            for i in range(self.__task_space):
                v_in_msg.data.append(v_in[i])
            v_in_pub.publish(v_in_msg)


if __name__ == '__main__':
    cont = AdmittanceController()
    cont.main()
