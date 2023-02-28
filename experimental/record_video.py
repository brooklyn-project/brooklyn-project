import argparse
import picamera
import time
import signal

VIDEO_DIR = '/home/pi/brooklyn-project/experimental/videos/'

parser = argparse.ArgumentParser(description='Record a video from the Raspberry Pi camera')
parser.add_argument('--preview', action='store_true', help='Show a preview of the camera')
parser.add_argument('--awb_mode', type=str, help='Set the auto white balance mode. Potential values: auto, sunlight, cloudy. Defaults to auto', default='auto')
parser.add_argument('duration', type=int, help='Duration of the video in seconds')

args = parser.parse_args()

timestamp = time.strftime("%m-%d-%H%M%S")

with picamera.PiCamera(sensor_mode=4) as camera:
    stream = picamera.PiCameraCircularIO(camera, seconds=3600)

    def save_recording(sig, frame):
        print("saving video")
        stream.copy_to(VIDEO_DIR + f'{timestamp}.h264')
        sys.exit(0)

    signal.signal(signal.SIGINT, save_recording)
    camera.awb_mode = args.awb_mode
    camera.resolution = (1296, 972)
    camera.framerate = 42
    if args.preview:
        camera.start_preview()
    print("STARTING RECORDING")
    camera.start_recording(stream, format='h264')

    try:
        while True:
            camera.wait_recording(1)
    except:
        print("STOPPING RECORDING")
        camera.stop_recording()

