import numpy as np


def getTiltDistortion(TILT, HEIGHT=10, FOV=np.radians(120), RESOLUTION=1024):
    """
    TILT -> Represented by 'phi' in documentation. Tilt of plane relative to horizon, right hand side above horizon is positive
    HEIGHT -> Height of camera relative to the ground in units of distance
    FOV -> Field of view of camera in radians
    RESOLUTION -> Resolution of image produced by camera in pixels
    
    Returns two arrays:
        pixel_indices -> Position in pixels of each pixel from 0 to RESOLUTION
        distances -> Ground distance from each pixel to the camera, corresponding to pixel_indices
    """
    dTheta = FOV/RESOLUTION
    leftAngle = 0.5*FOV - TILT
    rightAngle = 0.5*FOV + TILT
    
    leftThetas = np.arange(0, leftAngle, dTheta)
    rightThetas = np.linspace(0, rightAngle, RESOLUTION-leftThetas.shape[0]) # This possibly biases the left/right, but guarantees assertion below
    assert len(leftThetas) + len(rightThetas) == RESOLUTION

    leftDisplacements = dTheta*HEIGHT/ (np.cos(leftThetas))**2
    rightDisplacements = dTheta*HEIGHT/ (np.cos(rightThetas))**2

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
    
    pixel_indices = np.arange(0, RESOLUTION, 1)
    distances = np.zeros(RESOLUTION)
    for index in pixel_indices:
        if index < leftDistances.shape[0]:
            distances[index] = leftDistances[leftDistances.shape[0]-1-index]
        else:
            distances[index] = rightDistances[index-leftDistances.shape[0]]
    
    return pixel_indices, distances


# For plotting test output for this function:
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
