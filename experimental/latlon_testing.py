from PIL import Image
import numpy as np


def getTargetsViaColorMasking(image: np.array) -> list:
    """
    image -> Image to find targets in
    
    Returns:
        list of coordinates of targets in pixels relevant to image
    """
    threshold = 50
    target_color = np.array([26, 126, 244]) # Blue
    target_radius = 50

    image_height = image.shape[0]
    image_width = image.shape[1]
    
    color_mask = (np.sum(abs(image - target_color) < threshold, 2) == 3)
    
    # Plot targeting circles on top of possible targets
    init_targets = []
    target_sampling = 5
    for i in range(0, image_width, target_sampling):
        for j in range(0, image_height, target_sampling):
            if color_mask[j, i] == True:
                init_targets.append((i, j))

    # Compare against other targets already grouped
    target_groups = []
    for x1, y1 in init_targets:
        added_to_group = False
        for index in range(len(target_groups)):
            for x2, y2 in target_groups[index]:
                if pow(pow(x2-x1, 2) + pow(y2-y1, 2), 0.5) < target_radius*2:
                    target_groups[index].append((x1, y1))
                    added_to_group = True
                    break
            if added_to_group:
                break
        if not added_to_group:
            target_groups.append([(x1, y1)])

    # Average the groups to find the mean center point
    mean_targets = []
    for target_group in target_groups:
        mean_targets.append(np.mean(target_group, 0))
    
    return mean_targets

ALTITUDE = 61

DIAG_FOV = np.pi*(78/180)
HFOV = 2*np.arctan(np.tan(DIAG_FOV/2))*(img.shape[1]/img.shape[0])
VFOV = 2*np.arctan(np.tan(DIAG_FOV/2))*(img.shape[0]/img.shape[1])

img = np.array(Image.open("./sample_frames/892.png"))
pixel_coords = getTargetsViaColorMasking(img)

print(HFOV*180/np.pi)
print(VFOV*180/np.pi)

# print(VFOV/(360-HFOV))
# print(img.shape[1]/img.shape[0])