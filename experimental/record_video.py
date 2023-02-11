import argparse
import picamera
import time

VIDEO_DIR = '/home/pi/brooklyn-project/experimental/videos/'

parser = argparse.ArgumentParser(description='Record a video from the Raspberry Pi camera')
parser.add_argument('--preview', action='store_true', help='Show a preview of the camera')
parser.add_argument('duration', type=int, help='Duration of the video in seconds')

args = parser.parse_args()
timestamp = time.strftime("%m-%d-%H%M%S")

with picamera.PiCamera(sensor_mode=4) as camera:
    camera.resolution = (1296, 972)
    camera.framerate = 42
    if args.preview:
        camera.start_preview()
    camera.start_recording(VIDEO_DIR + f'{timestamp}.h264')
    camera.wait_recording(args.duration)
    camera.stop_recording()

