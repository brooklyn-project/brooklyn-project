import math


#Orientation assumes 0 degrees yaw is looking directly east
# Roll is x-direction phi
# Pitch is y-direction phi
# Diagnoal FOV is 120
# 4608 Horizontal pixels
# 2592 Vertical pixels
DiagFOV = 120




CAMERA_WIDTH_PIXELS = 4608
CAMERA_HEIGHT_PIXELS = 2592

XFOV = 120 * CAMERA_WIDTH_PIXELS / math.sqrt(CAMERA_HEIGHT_PIXELS**2 + CAMERA_WIDTH_PIXELS**2) 
YFOV = 120 * CAMERA_HEIGHT_PIXELS / math.sqrt(CAMERA_HEIGHT_PIXELS**2 + CAMERA_WIDTH_PIXELS**2)


def target_position(pixel_coords: list, pixhawk_lat: float, pixhawk_lon: float, pixhawk_altitude: float, yaw: float, pitch: float, roll: float):
    latlon_coordinates = []
    meter_coords = []
    dx2 = pixhawk_altitude * math.tan(XFOV/2 + roll)
    dx1 = pixhawk_altitude * math.tan(XFOV/2 - roll)
    dx = dx2+dx1
    dy2 = pixhawk_altitude * math.tan(YFOV/2 + pitch)
    dy1 = pixhawk_altitude * math.tan(YFOV/2 - pitch)
    dy = dy2+dy1

    for pair in pixel_coords:
        #Convert to terms of h then put in meters then convert to lat lon coords
    



    
    return latlon_coordinates