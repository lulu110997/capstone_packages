#!/usr/bin/env python
import unittest
import numpy as np
from ur_vogui_kdl import UrVoguiKdl

class TestUrVoguiKdl(unittest.TestCase):

    
    def test_jacobian(self):
        Robot = UrVoguiKdl(urdf_file='/home/louis/catkin_ws/src/custom_ur_control/rb_vogui_generated.urdf')
        
        q0 = np.array([0, 0, -0.87266, -1.2217, 1.2217, 0, 0, 0])

        j00 = np.array([
            [0.0000,    1.0000,    0.4343,    0.2957,   -0.0744,   -0.0744,    0.0593,    0.0000],
            [1.0000,    0.0000,    0.6988,   -0.3524,    0.0886,    0.0886,   -0.0706,    0.0000],
            [0.0000,    0.0000,    0.0000,   -0.7819,   -0.5723,    0.0000,   -0.0000,    0.0000],
            [0.0000,    0.0000,    0.0000,    0.7660,    0.7660,    0.7660,   -0.0000,    0.7660],
            [0.0000,    0.0000,   -0.0000,    0.6428,    0.6428,    0.6428,    0.0000,    0.6428],
            [0.0000,    0.0000,    1.0000,    0.0000,    0.0000,    0.0000,   -1.0000,    0.0000]
            ])


        je0 = np.array([
            [0.7660,   -0.6428,    0.2561,   -0.4600,    0.1157,    0.1157,   -0.0922,    0.0000],
            [0.0000,    0.0000,   -0.0000,   -0.7819,   -0.5723,    0.0000,    0.0000,    0.0000],
            [0.6428,    0.7660,    0.7819,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000],
            [0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000],
            [0.0000,    0.0000,    1.0000,    0.0000,    0.0000,    0.0000,   -1.0000,    0.0000],
            [0.0000,    0.0000,    0.0000,    1.0000,    1.0000,    1.0000,    0.0000,    1.0000]
            ])

        jacob00 = Robot.Jacob(q0)
        jacobe0 = Robot.Jacob(q0, ee_frame=True)

        for rows in range(j00.shape[0]):
            for cols in range(j00.shape[1]):
                self.assertAlmostEqual(jacob00[rows, cols], j00[rows,cols], delta=0.001)
                self.assertAlmostEqual(jacobe0[rows, cols], je0[rows,cols], delta=0.001)

        q1 = np.array([0.3, -1.2, 0, -0.5230, 1.2215, 2.9665, 1.5705, 0])

        j01 = np.array([
            [0.0000,    1.0000,   -0.1640,    0.0843,   -0.2217,    0.1463,    0.0000,    0.0000],
            [1.0000,    0.0000,    0.9470,    0.0000,    0.0000,    0.0000,   -0.0922,    0.0000],
            [0.0000,    0.0000,    0.0000,   -0.9470,   -0.4162,    0.0220,    0.0000,    0.0000],
            [0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.4998,   -0.8661],
            [0.0000,    0.0000,    0.0000,    1.0000,    1.0000,    1.0000,    0.0000,    0.0003],
            [0.0000,    0.0000,    1.0000,    0.0000,    0.0000,    0.0000,    0.8661,    0.4998]
            ])

        je1 = np.array([
            [1.0000,    0.0003,    0.9470,    0.0002,    0.0000,    0.0000,   -0.0922,    0.0000],
            [0.0000,   -0.4998,    0.0820,    0.7781,    0.4713,   -0.0922,   -0.0000,    0.0000],
            [0.0003,   -0.8661,    0.1423,   -0.5464,   -0.0160,   -0.1157,   -0.0000,    0.0000],
            [0.0000,    0.0000,   -0.0001,    1.0000,    1.0000,    1.0000,    0.0000,    0.0000],
            [0.0000,    0.0000,   -0.8661,   -0.0000,   -0.0000,   -0.0000,   -1.0000,    0.0000],
            [0.0000,    0.0000,    0.4998,    0.0003,    0.0003,    0.0003,    0.0000,    1.0000]
            ])

        jacob01 = Robot.Jacob(q1)
        jacobe1 = Robot.Jacob(q1, ee_frame=True)

        for rows in range(j01.shape[0]):
            for cols in range(j01.shape[1]):
                self.assertAlmostEqual(jacob01[rows, cols], j01[rows,cols], delta=0.001)
                self.assertAlmostEqual(jacobe1[rows, cols], je1[rows,cols], delta=0.001)

    def test_maniplty(self):
        Robot = UrVoguiKdl(urdf_file='/home/louis/catkin_ws/src/custom_ur_control/rb_vogui_generated.urdf')

        q0 = [0, 0, -0.87266, -1.2217, 1.2217, 0, 0, 0]
        m0 = 0
        self.assertAlmostEqual(m0, Robot.arm_maniplty(q0), delta=0.001)

        q1 = [0, 0, 0, -0.5230, 1.2215, 2.9665, 1.5705, 0]
        m1 = 0.3383
        self.assertAlmostEqual(m1, Robot.arm_maniplty(q1), delta=0.001)

        q2 = [-0.1, 2.12, -0.0893, -0.6060, 0.3392, 0.3601, 0.8232, -0.0629]
        m2 = 0.0894
        self.assertAlmostEqual(m2, Robot.arm_maniplty(q2), delta=0.001)

        q3 = [1.56, -0.8, -0.2513, -1.2566, 1.2566, -1.5708, 1.5708, -1.5708]
        m3 = 0.2926
        self.assertAlmostEqual(m3, Robot.arm_maniplty(q3), delta=0.001)

if __name__ == '__main__':
    unittest.main()