#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64MultiArray
import numpy as np

# Global
C_D = 0
RHO = 0
A = 0
M = 0 
G = 9.81


def solve_ODE(v_x, v_y, v_z):
    # Get wind update
    # w_1 / w_2 = (z_1 / z_2)^p
    # w_1 = wind at height z_1
    # w_2 = wind at height z_2
    # p = exponential parameter

    w_x, w_y, w_z = 0, 0, 0

    # Update wind estimate and airspeed V_r 
    v_r = np.sqrt((v_x - w_x)**2 + (v_y - w_y)**2 + (v_z - w_z)**2)

    # Solving ODE
    h = 0.5 # Set size

    v_x_dot = (-C_D*RHO*A)/(2*M)*(v_x - w_x)*v_r
    v_y_dot = (-C_D*RHO*A)/(2*M)*(v_y - w_y)*v_r
    v_z_dot = G - (C_D*RHO*A)/(2*M)*(v_z - w_z)*v_r

    x, y, z = 0, 0, 0
    return x, y, z, v_x, v_y, v_z


def bal_Calc(V, true_course):
    # Drop altitude, initial z value
    z = 400
    x_curr, y_curr, z_curr = 0, 0, 0

    # Aircraft speed component
    v_x = V*np.cos(true_course)
    v_y = V*np.sin(true_course)
    v_z = 0

    while z > 0:
        x_prev, y_prev, z_prev = x_curr, y_curr, z_curr

        x_curr, y_curr, z_curr, v_x, v_y, v_z = solve_ODE(v_x, v_y, v_z)
        


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " ~ %s", data.data)

def predict_drop():
    

    rospy.spin()

if __name__ == '__main__':  
    predict_drop()