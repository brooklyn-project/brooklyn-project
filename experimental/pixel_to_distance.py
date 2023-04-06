import numpy as np
import math


def calculatePixelDistances(tilt, height, FOV, resolution):
    """
    tilt -> Represented by 'phi' in documentation. tilt of plane relative to horizon, right hand side above horizon is positive
    height -> Height of camera relative to the ground in units of distance
    FOV -> Field of view of camera in radians
    resolution -> Resolution of image produced by camera in pixels
    
    Returns two arrays:
        pixel_indices -> Position in pixels of each pixel from 0 to 'resolution'
        distances -> Ground distance from each pixel to the camera, corresponding to 'pixel_indices'
    """
    dTheta = FOV/resolution
    leftAngle = 0.5*FOV - tilt
    rightAngle = 0.5*FOV + tilt
    
    leftThetas = np.arange(0, leftAngle, dTheta)
    rightThetas = np.linspace(0, rightAngle, resolution-leftThetas.shape[0]) # This possibly biases the left/right, but guarantees assertion below
    assert len(leftThetas) + len(rightThetas) == resolution

    leftDisplacements = dTheta*height/ (np.cos(leftThetas))**2
    rightDisplacements = dTheta*height/ (np.cos(rightThetas))**2

    # Calculate distances from directly below the plane
    leftDistances = np.zeros(leftDisplacements.shape[0])
    for index in range(leftDisplacements.shape[0]):
        sum_of_disps = 0
        for sum_index in range(index+1):
            sum_of_disps += leftDisplacements[sum_index]
        leftDistances[index] = sum_of_disps

    rightDistances = np.zeros(rightDisplacements.shape[0])
    for index in range(rightDisplacements.shape[0]):
        sum_of_disps = 0
        for sum_index in range(index+1):
            sum_of_disps += rightDisplacements[sum_index]
        rightDistances[index] = sum_of_disps
    
    pixel_indices = np.arange(0, resolution, 1)
    distances = np.zeros(resolution)
    for index in pixel_indices:
        if index < leftDistances.shape[0]:
            distances[index] = leftDistances[leftDistances.shape[0]-1-index]
        else:
            distances[index] = rightDistances[index-leftDistances.shape[0]]
    
    return pixel_indices, distances


def addMetersToCoords(latitude, longitude, dlat, dlon):
    """
    latitude -> Latitude coordinate to add to
    longitude -> Longitude coordinate to add to
    dlat -> Distance in meters to add to latitude coordinate
    dlon -> Distance in meters to add to longitude coordinate
    
    Returns two floats:
        -> New latitude
        -> New longitude
    """
    R_EARTH = 6378000
    new_lat  = latitude  + (dlat / R_EARTH) * (180 / np.pi)
    new_lon = longitude + (dlon / R_EARTH) * (180 / np.pi) / np.cos(latitude * np.pi/180)
    
    return new_lat, new_lon
    

def getTargetLatLon(plane_lat, plane_lon, plane_pitch, plane_roll, plane_yaw, plane_altitude, target_x, target_y, image_x, image_y):
    """
    plane_lat -> Latitude coordinate of the plane
    plane_lon -> Longitude coordinate of the plane
    plane_pitch -> Pitch of plane in radians
    plane_roll -> Roll of plane in radians
    plane_yaw -> Yaw of plane in radians
    plane_altitude -> altitude of plane relative to the ground in meters
    target_x -> Position of target along x-dimension of image in pixels
    target_y -> Position of target along y-dimension of image in pixels
    image_x -> Width of camera image of target in pixels
    image_y -> height of camera image of target in pixels
    
    Returns two floats:
        target_lat -> Latitude coordinate of the target of interest
        target_lon -> Longitude coordinate of the target of interest
    """

    # This assumes that the yaw degree of cardinal north is 0 degrees. Change this if we test and it's different than what we thought
    NORTHYAW =0


    DiagFOV = 120
    

    XFOV = 120 * image_x / math.sqrt(image_y**2 + image_x**2)  * math.pi() / 180
    YFOV = 120 * image_y / math.sqrt(image_y**2 + image_x**2) * math.pi() / 180

    x_distances = calculatePixelDistances(plane_roll, plane_altitude, XFOV, image_x)
    y_distances = calculatePixelDistances(plane_pitch, plane_altitude, YFOV, image_y)
    
    x_pixel_index = x_distances[0].index(target_x)
    x_dist = x_distances[1][x_pixel_index]
    y_pixel_index = y_distances[0].index(target_y)
    y_dist = y_distances[1][y_pixel_index]

    yaw = plane_yaw - NORTHYAW

    x_prime_meters = x_dist*math.cos(yaw) + y_dist*math.sin(yaw)
    y_prime_meters = y_dist*math.cos(yaw) - x_dist*math.sin(yaw)

    finalCoords = addMetersToCoords(plane_lat, plane_lon, y_prime_meters, x_prime_meters)

    target_lat = finalCoords[0]
    target_lon = finalCoords[1]

    return target_lat, target_lon


# For plotting test output of calculatePixelDistances function:
#
# import matplotlib.pyplot as plt
# RESOLUTION = 2000

# f, ax1 = plt.subplots(1, 1, figsize=(10, 8), facecolor='w')
# lw=2

# for phi, color in [[0, "Blue"], [5, "Red"], [10, "Green"], [15, "Orange"], [20, "Purple"]]:
#     pixels, distances = getTiltDistortion(np.radians(phi), RESOLUTION=RESOLUTION)
    
#     ax1.plot(pixels, distances, color=color, lw=lw, label=f"{phi}°")

# f.suptitle(f"Pixel Distances at Various Camera Tilt Angles", fontsize=25)
# ax1.tick_params(axis='both', which='major', labelsize=20)
# ax1.legend(fontsize=15)
# ax1.set_xlabel("Pixel Position in Image", fontsize=20)
# ax1.set_ylabel("Distance from Camera (m)", fontsize=20)
# ax1.set_xlim(0, RESOLUTION)
# ax1.grid()
