#!/usr/bin/env python
import rospy
# from geometry_msgs import PoseWithCovarianceStamped, TwistStamped
from std_msgs.msg import String, Float64MultiArray, Float64
import numpy as np
import math

# global variables
G = 9.81

# commanded lateral acceleration that brings the UAV closer to the desired path
# V_g ~ [vx, vy, vz] ground speed
# L1 ~ tunable length of a vector, starting from the UAV, which intersects the desired path in front of the UAV
# n ~ angle between the this vector and the velocity vector of the UAV
def cmd_lat_accel(V_g, L1, n):
    a_cmd = 2 * np.linalg.norm(V_g)**2/L1 * math.sin(n)

    return a_cmd

# desired roll angle
# theta ~ pitch angle
# a_cmd ~ commanded lateral acceleration
def roll_angle(a_cmd, theta):
    phi_d = math.cos(theta) * math.arctan(a_cmd/G)

    return phi_d

# pitch angle calculation
def pitch(gam, gam_d, alpha_trim):
    c1 = 0 # constant
    theta_d = -c1 * (gam - gam_d) + gam_d + alpha_trim
    
    return theta_d

# main function
if __name__ == '__main__': 
    print("hi")