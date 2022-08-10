#!/usr/bin/env python3.6
from math import pi as PI
import roboticstoolbox as rtb
import spatialmath.base as tr
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import numpy as np

class Robot_with_no_prismatic(DHRobot):
    def __init__(self):
        scale_lengths = 1.25
        L1 = RevoluteDH(d=0, a=scale_lengths*0.6127, alpha=0, offset=-PI/2)
        L2 = RevoluteDH(d=0, a=scale_lengths*0.5723, alpha=0)
        L3 = RevoluteDH(d=0, a=scale_lengths*0.4, alpha=0)
        L4 = RevoluteDH(d=0, a=scale_lengths*0.5283, alpha=0)
        L5 = RevoluteDH(d=0, a=scale_lengths*0.6, alpha=0)

        super().__init__(
                [
                    L1,
                    L2,
                    L3,
                    L4,
                    L5
                ], name="Robot"
                        )

class Robot_with_prismatic(DHRobot):
    def __init__(self):
        scale_lengths = 1.25
        P1 = PrismaticDH(theta=-PI/2, a=0, alpha=-PI/2, qlim=[-2,2], offset=0); # PRISMATIC Link along y
        P2 = PrismaticDH(theta=-PI/2, a=0, alpha=-PI/2, qlim=[-2,2], offset=0); # PRISMATIC Link along x
        L1 = RevoluteDH(d=0, a=scale_lengths*0.6127, alpha=0, offset=PI)
        L2 = RevoluteDH(d=0, a=scale_lengths*0.5723, alpha=0)
        L3 = RevoluteDH(d=0, a=scale_lengths*0.4, alpha=0)
        L4 = RevoluteDH(d=0, a=scale_lengths*0.5283, alpha=0)
        L5 = RevoluteDH(d=0, a=scale_lengths*0.6, alpha=0)

        super().__init__(
                [
                    P1,
                    P2,
                    L1,
                    L2,
                    L3,
                    L4,
                    L5
                ], name="Robot"
                        )

NO_OPTIMISATION = 1

R = Robot_with_no_prismatic()

# R = Robot_with_prismatic()
# R.base = tr.trotx(-PI/2)

# R.teach([0, 0, 0.2, 0.4398,    0.3770,    1.2566,    0.4398])
# jacobian = R.jacob0([0, 0, 0.2000,    0.4398,    0.3770,    1.2566,    0.4398])
# jacobian = R.jacob0([0.2000,    0.4398,    0.3770,    1.2566,    0.4398])
# np.set_printoptions(suppress=True, precision=4)
# print(f"{jacobian}")
#     0.4786   -0.2720   -0.8459   -1.1090   -0.6822
#     1.8200    1.6678    1.2407    0.8155    0.3116
#          0         0         0         0         0
#          0         0         0         0         0
#          0         0         0         0         0
#     1.0000    1.0000    1.0000    1.0000    1.0000
