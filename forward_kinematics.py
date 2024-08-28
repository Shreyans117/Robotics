import numpy as np
from math import pi, cos, sin
import modern_robotics as mr
 
def forward_kinematics(joints):
    # input: joint angles [joint1, joint2, joint3]
    # output: the position of end effector [x, y, z]
    # add your code here to complete the computation
 
    link1z = 0.065
    link2z = 0.039
    link3x = 0.050
    link3z = 0.150
    link4x = 0.150
    joint1 = joints[0]
    joint2 = joints[1]
    joint3 = joints[2]
 
    # Easier to read the links
    l1 = link1z
    l3x = link3x
    l3z = link3z
    l2 = link2z
    l4 = link4x
 
    # Transformation from origin to J1
    T_0_J1 = np.matrix([[cos(joint1), -sin(joint1), 0, 0],
                        [sin(joint1), cos(joint1), 0, 0],
                        [0, 0, 1, l1],
                        [0, 0, 0, 1]])
 
    # Transformation from J1 -> J2
    T_J1_J2 = np.matrix([[cos(joint2), 0, sin(joint2), 0],
                         [0, 1, 0, 0],
                         [-sin(joint2), 0, cos(joint2), l2],
                         [0, 0, 0, 1]])
 
    # Rotate about -ve y axis & Translate
    T_J2_J3 = np.matrix([[1, 0, 0, l3x],
                        [0, -1, 0, 0],
                        [0, 0, -1, l3z],
                        [0, 0, 0, 1]])
   
    # Rotate J3 by joint3 angle
    T_J3_R = np.matrix([[cos(joint3), 0, sin(joint3), 0],
                        [0, -1, 0, 0],
                        [-sin(joint3), 0, cos(joint3), 0],
                        [0, 0, 0, 1]])
 
    # Transformation from J3 to J4
    T_J3_J4 = np.matrix([[1, 0, 0, l4],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0],
                        [0, 0, 0, 1]])
 
    # Perform transformation
    T = T_0_J1 * T_J1_J2 * T_J2_J3 * T_J3_R * T_J3_J4
 
    # Extract x, y and z
    x = T.item(0, 3)
    y = T.item(1, 3)
    z = T.item(2, 3)
 
    return [x, y, z]
