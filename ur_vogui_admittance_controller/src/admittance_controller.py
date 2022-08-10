#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import WrenchStamped

class AdmittanceController(object):
    '''
    An admittance controller for a robotic manipulator
    '''

    def __init__(self):
        rospy.init_node('admittance_controller')

        # Attributes
        self.__task_space = 6
        self.__ft = np.zeros((self.__task_space,1)) # Force exerted on the ee (from joystick)
        self._deadman_switch = [0, 0] # First element will contain the time the msg was received and second element contains the state of button

        # Subscriber. Use either joystick or force torque sensor for admittance control
        self._joystick = rospy.get_param("joystick", 1)

        if self._joystick:
            rospy.Subscriber("joy", Joy, self.joy_callback) # For obtaining joystick values
        else:
            rospy.Subscriber("wrench_out", WrenchStamped, self.force_callback) # For obtaining force exerted on EE
            rospy.Subscriber("joy", Joy, self.deadman_switch_cb) # Joystick button acts as a deadman switch


    def deadman_switch_cb(self, msg):
        '''
        'A' button of the xbox joystick acts as the deadman switch which stops the admittance
        controller when not pressed
        '''
        self._deadman_switch[0] = msg.header.stamp
        self._deadman_switch[1] = msg.buttons[0]

    def force_callback(self, msg):
        '''
        Obtain force/torque input by operator
        '''
        self.__ft[0] = msg.wrench.force.x
        self.__ft[1] = msg.wrench.force.y
        self.__ft[2] = msg.wrench.force.z
        self.__ft[5] = msg.wrench.torque.z

    def joy_callback(self, msg):
        '''
        Obtain the current values on the joysticke
        '''
        self.__ft[0] = msg.axes[0] # Left right of left joystick mapped to x-axis transl
        self.__ft[1] = msg.axes[1] # Up down of left joystick mapped to y-axis transl
        self.__ft[2] = msg.axes[4] # Left right of right joystick mapped to z-axis rot
        self.__ft[5] = msg.axes[3] # Up down of right joystick mapped to z-axis transl
        self._deadman_switch[0] = msg.header.stamp
        self._deadman_switch[1] = msg.buttons[0]

    def main(self):
        '''
        Main thread that continually moves the robot arm based on the joystick input
        '''

        # Publisher for desired velocity in the cartesian space
        v_in_pub = rospy.Publisher("velocity_input", Float64MultiArray, queue_size=1)

        # Populate the virtual damping of the admittance controller
        kd = np.zeros((self.__task_space,self.__task_space))

        if self._joystick:
            damping_manip_transl = 6.0
            damping_manip_rot = 6.0
        else:
            damping_manip_transl = 220.0
            damping_manip_rot = 15.0

        for i in range(self.__task_space):
            if i < 3:
                kd[i,i] = 1.0 / damping_manip_transl
            else:
                kd[i,i] = 1.0 / damping_manip_rot

        rospy.loginfo("The damping matrix for the admittance controller is:\n" + str(kd))

        rate_limiter = rospy.Rate(300) # Ideally, control loop should run the same rate that the HARDWARE is running at

        rospy.sleep(1.0)
        rospy.loginfo("Entering control loop of admittance controller...")

        while not rospy.is_shutdown():
            # 'A' needs to be pressed and the joy msg must have been received within 0.1s 
            # for the admittance controller to activate
            time_diff = rospy.Time.now() - self._deadman_switch[0]

            # Initialise the velocity output of controller
            v_in_msg = Float64MultiArray()
            v_in_msg.data = self.__task_space*[0]

            if (time_diff.to_sec() < 0.01) and self._deadman_switch[1]:
                v_in = np.matmul(kd, self.__ft) # Convert force to velocity input

                # Convert np to ros_msg and publish the desired cartesian velocity over a topic
                for i in range(self.__task_space):
                    v_in_msg.data[i] = v_in[i]
                v_in_pub.publish(v_in_msg)
            else:
                # Publish a msg with zero elements
                v_in_pub.publish(v_in_msg)

            rate_limiter.sleep()


if __name__ == '__main__':
    cont = AdmittanceController()
    cont.main()
