#!/usr/bin/env python
import rospy
from geometry_msgs import PoseWithCovarianceStamped, TwistStamped
from std_msgs.msg import String, Float64MultiArray, Float64
import numpy as np
import math

# GLOBAL VARIABLES (CONSTANT THROUGHOUT FLIGHT)
C_D = 0.05 # MISSING
RHO = 1.293 # from literature (kg m^−3)
A = 0.05 # MISSING, area exposed to direction of drop (m^2)
M = 0.1133981 # payload mass (kg) 0.25 lbs
G = 9.81 # gravity constant (m/s^2)
P = 1/7 # exponential parameter from wind update

# Updating the wind at each time step with wind profile power law
def wind_Update(w_1, z_1, z_2):
    rhs = (z_1 / z_2)**(P)
    w_2 = w_1 / rhs

    return w_2

# Solving the ODE 
def solve_ODE(v_x, v_y, v_z, w_x, w_y, w_z, x_disp, y_disp, z_loc):

    # Update wind estimate and airspeed V_r 
    v_r = np.sqrt((v_x - w_x)**2 + (v_y - w_y)**2 + (v_z - w_z)**2)

    # Solving ODE
    h = 0.05 # Set size

    x_disp += v_x*h
    y_disp += v_y*h
    z_loc -= v_z*h

    v_x_dot = (-C_D*RHO*A)/(2*M)*(v_x - w_x)*v_r
    v_y_dot = (-C_D*RHO*A)/(2*M)*(v_y - w_y)*v_r
    v_z_dot = G - (C_D*RHO*A)/(2*M)*(v_z - w_z)*v_r

    v_x += v_x_dot*h
    v_y += v_y_dot*h
    v_z += v_z_dot*h

    return x_disp, y_disp, z_loc, v_x, v_y, v_z

# Determine location of drop for payload
def bal_Calc(target_lat, target_long, v_x, v_y, v_z, z_loc, w_x, w_y, w_z):
    # Drop altitude, initial z value
    z_curr = z_loc
    w_x_curr, w_y_curr, w_z_curr = w_x, w_y, w_z

    # x and y displacement
    x_disp, y_disp =  0, 0

    # Aircraft speed component
    # v_x = V*np.sin(true_course*math.pi/180)
    # v_y = V*np.cos(true_course*math.pi/180)

    # First step 
    z_prev = z_curr
    x = []
    y = []
    z = []

    x_disp, y_disp, z_curr, v_x, v_y, v_z = solve_ODE(v_x, v_y, v_z, w_x_curr, w_y_curr, w_z_curr, x_disp, y_disp, z_curr)

    while z_curr > 0:
        # Get wind update
        w_x_curr = wind_Update(w_x_curr, z_prev, z_curr)
        w_y_curr = wind_Update(w_y_curr, z_prev, z_curr)
        w_z_curr = wind_Update(w_z_curr, z_prev, z_curr)

        # print(w_x_curr, w_y_curr, w_z_curr)

        # Set previous z coordinate to previous variable
        z_prev = z_curr

        # Solve ODE for new x,y,z coordinate and velocity
        x_disp, y_disp, z_curr, v_x, v_y, v_z = solve_ODE(v_x, v_y, v_z, w_x_curr, w_y_curr, w_z_curr, x_disp, y_disp, z_curr)
        # print(x_disp, y_disp, z_curr, v_x, v_y, v_z)
        # print(x_disp, y_disp, z_curr)
        x.append(x_disp)
        y.append(y_disp)
        z.append(z_curr)
        
    # Release coordinates are equal to the target coordinates, shifted 
    # − x m in the north direction and − y m in the east direction
    return x_disp, y_disp, z_curr, x, y, z
    

# ROS 
class BallisticsInfo:
    def __init__(self):
        rospy.init_node('predict_drop', anonymous=True)

        self.latitude = 0
        self.longitude = 0
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0
        self.heading_data = 0
        self.altitude_data = 0
        self.wind_x = 0
        self.wind_y = 0
        self.wind_z = 0

        rospy.Subscriber("/mavros/global_position/local", PoseWithCovarianceStamped, self.location_callback)
        rospy.Subscriber("/mavros/global_position/gp_vel", TwistStamped, self.velocity_callback)
        rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.heading_callback)
        rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.altitude_callback)
        rospy.Subscriber("/mavros/global_position/wind_estimation", TwistStamped, self.wind_callback)

    def location_callback(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude

    def velocity_callback(self, data):
        self.vel_x = data.twist.linear.x
        self.vel_y = data.twist.linear.y
        self.vel_z = data.twist.linear.z

    def heading_callback(self, data):
        self.heading_data = data.data

    def altitude_callback(self, data):
        self.altitude_data = data.data

    def wind_callback(self, data):
        self.wind_x = data.wind.x
        self.wind_y = data.wind.y
        self.wind_z = data.wind.z

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Calculate ballistics drop location
            x_disp, y_disp, z_curr, x, y, z = bal_Calc(self.latitude, self.longitude, self.vel_x, self.vel_y, self.vel_z, self.altitude_data, self.wind_x, self.wind_y, self.wind_z)
            rate.sleep()

# Main functions
if __name__ == '__main__':  
    # target_lat = 0
    # target_long = 0
    # V = 40
    # true_course = 90
    # z_loc = 400
    # w_x, w_y, w_z = 0,0,0

    # x_disp, y_disp, z_curr, x, y, z = bal_Calc(target_lat, target_long, V, true_course, z_loc, w_x, w_y, w_z)
    # print(x_disp, y_disp, z_curr)

    listener = BallisticsInfo()
    listener.spin()

