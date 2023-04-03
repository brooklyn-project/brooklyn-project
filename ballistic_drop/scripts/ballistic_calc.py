#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64MultiArray
import numpy as np

# GLOBAL VARIABLES (CONSTANT THROUGHOUT FLIGHT)
C_D = 0 
RHO = 0 
A = 1 # area exposed to direction of drop
M = 2 # payload mass
G = 9.81 # gravity constant 
P = 1/7 # exponential parameter from wind update

# Updating the wind at each time step with wind profile power law
def wind_Update(w_1, z_1, z_2):
    rhs = (z_1 / z_2)**(P)
    w_2 = w_1 / rhs

    return w_2

# Solving the ODE 
def solve_ODE(v_x, v_y, v_z, w_x, w_y, w_z):

    # Update wind estimate and airspeed V_r 
    v_r = np.sqrt((v_x - w_x)**2 + (v_y - w_y)**2 + (v_z - w_z)**2)

    # Solving ODE
    h = 0.5 # Set size

    v_x_dot = (-C_D*RHO*A)/(2*M)*(v_x - w_x)*v_r
    v_y_dot = (-C_D*RHO*A)/(2*M)*(v_y - w_y)*v_r
    v_z_dot = G - (C_D*RHO*A)/(2*M)*(v_z - w_z)*v_r

    x, y, z = 0, 0, 0
    return x, y, z, v_x, v_y, v_z


def bal_Calc(target_lat, target_long, V, true_course, z_loc, w_x, w_y, w_z):
    # Drop altitude, initial z value
    z_curr = z_loc
    w_x_curr, w_y_curr, w_z_curr = w_x, w_y, w_z

    # Aircraft speed component
    v_x = V*np.cos(true_course)
    v_y = V*np.sin(true_course)
    v_z = 0

    # First step 
    z_prev = z_curr

    x_curr, y_curr, z_curr, v_x, v_y, v_z = solve_ODE(v_x, v_y, v_z, w_x_curr, w_y_curr, w_z_curr)

    while z_curr > 0:
        # Get wind update
        w_x_curr = wind_Update(w_x_curr, z_prev, z_curr)
        w_y_curr = wind_Update(w_y_curr, z_prev, z_curr)
        w_z_curr = wind_Update(w_z_curr, z_prev, z_curr)

        # Set previous z coordinate to previous variable
        z_prev = z_curr

        # Solve ODE for new x,y,z coordinate and velocity
        x_disp, y_disp, z_curr, v_x, v_y, v_z = solve_ODE(v_x, v_y, v_z, w_x_curr, w_y_curr, w_z_curr)
        
    # Release coordinates are equal to the target coordinates, shifted 
    # − x m in the north direction and − y m in the east direction

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " ~ %s", data.data)

def predict_drop():
    

    rospy.spin()

if __name__ == '__main__':  
    target_lat = 0
    target_long = 0
    V = 40
    true_course = 0
    z_loc = 400
    w_x, w_y, w_z = 1,1,0

    bal_Calc(target_lat, target_long, V, true_course, z_loc, w_x, w_y, w_z)
    # predict_drop()