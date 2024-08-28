import numpy as np
from math import pi, cos, sin, atan, atan2, sqrt, acos

def inverse_kinematics(position):
    # input: the position of end effector [x, y, z]
    # output: joint angles [joint1, joint2, joint3]
    # add your code here to complete the computation

    link1z = 0.065
    link2z = 0.039
    link3x = 0.050
    link3z = 0.150
    link4x = 0.150
    x = position[0]
    y = position[1]
    z = position[2]
    x0=x
    x=sqrt(x**2+y**2)
    
    gamma=atan((z-link1z-link2z)/x)
    z1=(z-link1z-link2z)
    alpha=atan(link3x/link3z)
    j2j3=sqrt(link3x**2+link3z**2)
    
    j2j4=sqrt(x**2+z1**2)
    j3j4=link4x
    beta=acos((j2j3**2+j2j4**2-j3j4**2)/(2*j2j3*j2j4))
    epsilon=acos((j3j4**2+j2j4**2-j2j3**2)/(2*j3j4*j2j4))
    
    if(x0==0):
        theta1=pi/2
    else:
        theta1=atan2(y,x0)

    theta2=pi/2-alpha-beta-gamma
    theta5=pi/2-alpha
    theta3dash=pi-theta5
    theta3=pi-theta3dash-epsilon-beta
    joint1=theta1
    joint2=theta2
    joint3=theta3
    return [joint1, joint2, joint3]
