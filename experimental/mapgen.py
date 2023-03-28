import matplotlib.pyplot as plt

LAT_TOP_LEFT = 30.328463888888887
LONG_TOP_LEFT = -97.60950555555554

LAT_BOT_RIGHT = 30.312166666666666
LONG_BOT_RIGHT = -97.5937611111111

ORIGINAL_IMAGE_WIDTH = 10224 
ORIGINAL_IMAGE_HEIGHT = 11607

IMAGE_WIDTH = 2552
IMAGE_HEIGHT = 2724

TOP_LEFT_CORNER_PIXEL_WIDTH = 2600
TOP_LEFT_CORNER_PIXEL_HEIGHT = 1800

BOTTOM_RIGHT_CORNER_PIXEL_WIDTH = TOP_LEFT_CORNER_PIXEL_WIDTH + IMAGE_WIDTH
BOTTOM_RIGHT_CORNER_PIXEL_WIDTH = TOP_LEFT_CORNER_PIXEL_HEIGHT + IMAGE_HEIGHT

LAT_PER_PIXEL = (LAT_BOT_RIGHT-LAT_TOP_LEFT)/ORIGINAL_IMAGE_HEIGHT
LONG_PER_PIXEL = (LONG_BOT_RIGHT-LONG_TOP_LEFT)/ORIGINAL_IMAGE_WIDTH

LON_TL = TOP_LEFT_CORNER_PIXEL_WIDTH*LONG_PER_PIXEL + LONG_TOP_LEFT
LAT_TL = TOP_LEFT_CORNER_PIXEL_HEIGHT*LAT_PER_PIXEL + LAT_TOP_LEFT

LON_BR = LON_TL + IMAGE_WIDTH*LONG_PER_PIXEL
LAT_BR = LAT_TL + IMAGE_HEIGHT*LAT_PER_PIXEL



def lat_to_y(latitude: float):
    return latitude

def lat_to_x(longitude: float):
    return longitude

print(LAT_BR)
print(LON_BR)

print(LAT_TL)
print(LON_TL)