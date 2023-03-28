from PIL import Image
from matplotlib import pyplot as plt

# Coordinates of the corners of the image
corner_coordinates = [
    (40.1234, -75.4567),  # Top left corner
    (40.1234, -75.1234),  # Top right corner
    (39.7890, -75.4567),  # Bottom left corner
    (39.7890, -75.1234)   # Bottom right corner
]

# GPS coordinates of the 5 points to plot
points = [
    (40.0000, -75.3000),
    (40.0500, -75.4000),
    (40.1000, -75.5000),
    (40.1500, -75.6000),
    (40.2000, -75.7000)
]

# Open the JPEG image
image = Image.open('image.jpg')

# Get the width and height of the image
width, height = image.size

# Calculate the pixel coordinates of the corner GPS coordinates
pixel_corners = []
for lat, lon in corner_coordinates:
    x = int((lon - corner_coordinates[0][1]) / (corner_coordinates[1][1] - corner_coordinates[0][1]) * width)
    y = int((lat - corner_coordinates[0][0]) / (corner_coordinates[2][0] - corner_coordinates[0][0]) * height)
    pixel_corners.append((x, y))

# Calculate the pixel coordinates of the 5 points to plot
pixel_points = []
for lat, lon in points:
    x = int((lon - corner_coordinates[0][1]) / (corner_coordinates[1][1] - corner_coordinates[0][1]) * width)
    y = int((lat - corner_coordinates[0][0]) / (corner_coordinates[2][0] - corner_coordinates[0][0]) * height)
    pixel_points.append((x, y))

# Plot the image and the points
plt.imshow(image)
plt.plot([p[0] for p in pixel_points], [p[1] for p in pixel_points], 'ro')
plt.show()